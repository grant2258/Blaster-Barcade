/***************************************************************************

  ay8910.c


  Emulation of the AY-3-8910 / YM2149 sound chip.

  Based on various code snippets by Ville Hallik, Michael Cuddy,
  Tatsuyuki Satoh, Fabrice Frances, Nicola Salmoria.
  Mostly rewritten by couriersud in 2008

  TODO:
  * The AY8930 has an extended mode which is currently
    not emulated.
  * The YMZ284 only has one 1 output channel (mixed chan A,B,C).
    This should be forced.
  * YM2610 & YM2608 will need a separate flag in their config structures
    to distinguish between legacy and discrete mode.

  The rewrite also introduces a generic model for the DAC. This model is
  not perfect, but allows channel mixing based on a parametrized approach.
  This model also allows to factor in different loads on individual channels.
  If a better model is developped in the future or better measurements are
  available, the driver should be easy to change. The model is described
  later.

  In order to not break hundreds of existing drivers by default the flag
  AY8910_LEGACY_OUTPUT is used by drivers not changed to take into account the
  new model. All outputs are normalized to the old output range (i.e. 0 .. 7ffff).
  In the case of channel mixing, output range is 0...3 * 7fff.

  The main difference between the AY-3-8910 and the YM2149 is, that the
  AY-3-8910 datasheet mentions, that fixed volume level 0, which is set by
  registers 8 to 10 is "channel off". The YM2149 mentions, that the generated
  signal has a 2V DC component. This is confirmed by measurements. The approach
  taken here is to assume the 2V DC offset for all outputs for the YM2149.
  For the AY-3-8910, an offset is used if envelope is active for a channel.
  This is backed by oscilloscope pictures from the datasheet. If a fixed volume
  is set, i.e. enveloppe is disabled, the output voltage is set to 0V. Recordings
  I found on the web for gyruss indicate, that the AY-3-8910 offset should
  be around 0.2V. This will also make sound levels more compatible with
  user observations for scramble.

  The Model:
                     5V     5V
                      |      |
                      /      |
  Volume Level x >---|       Z
                      >      Z Pullup Resistor RU
                       |     Z
                       Z     |
                    Rx Z     |
                       Z     |
                       |     |
                       '-----+-------->  >---+----> Output signal
                             |               |
                             Z               Z
               Pulldown RD   Z               Z Load RL
                             Z               Z
                             |               |
                            GND             GND

Each Volume level x will select a different resistor Rx. Measurements from fpgaarcade.com
where used to calibrate channel mixing for the YM2149. This was done using
a least square approach using a fixed RL of 1K Ohm.

For the AY measurements cited in e.g. openmsx as "Hacker Kay" for a single
channel were taken. These were normalized to 0 ... 65535 and consequently
adapted to an offset of 0.2V and a VPP of 1.3V. These measurements are in
line e.g. with the formula used by pcmenc for the volume: vol(i) = exp(i/2-7.5).

The following is documentation from the code moved here and amended to reflect
the changes done:

Careful studies of the chip output prove that the chip counts up from 0
until the counter becomes greater or equal to the period. This is an
important difference when the program is rapidly changing the period to
modulate the sound. This is worthwhile noting, since the datasheets
say, that the chip counts down.
Also, note that period = 0 is the same as period = 1. This is mentioned
in the YM2203 data sheets. However, this does NOT apply to the Envelope
period. In that case, period = 0 is half as period = 1.

Envelope shapes:
    C AtAlH
    0 0 x x  \___
    0 1 x x  /___
    1 0 0 0  \\\\
    1 0 0 1  \___
    1 0 1 0  \/\/
    1 0 1 1  \```
    1 1 0 0  ////
    1 1 0 1  /```
    1 1 1 0  /\/\
    1 1 1 1  /___

The envelope counter on the AY-3-8910 has 16 steps. On the YM2149 it
has twice the steps, happening twice as fast.

***************************************************************************/

#include "driver.h"
#include "ay8910.h"

/*************************************
 *
 *  Defines
 *
 *************************************/

#define MAX_OUTPUT 0x7fff
#define NUM_CHANNELS 3

int ay8910_index_ym;
static int num = 0, ym_num = 0;


/* register id's */
#define AY_AFINE	(0)
#define AY_ACOARSE	(1)
#define AY_BFINE	(2)
#define AY_BCOARSE	(3)
#define AY_CFINE	(4)
#define AY_CCOARSE	(5)
#define AY_NOISEPER	(6)
#define AY_ENABLE	(7)
#define AY_AVOL		(8)
#define AY_BVOL		(9)
#define AY_CVOL		(10)
#define AY_EFINE	(11)
#define AY_ECOARSE	(12)
#define AY_ESHAPE	(13)

#define AY_PORTA	(14)
#define AY_PORTB	(15)

#define NOISE_ENABLEQ(_psg, _chan)	(((_psg)->regs[AY_ENABLE] >> (3 + _chan)) & 1)
#define TONE_ENABLEQ(_psg, _chan)	(((_psg)->regs[AY_ENABLE] >> (_chan)) & 1)
#define TONE_PERIOD(_psg, _chan)	( (_psg)->regs[(_chan) << 1] | (((_psg)->regs[((_chan) << 1) | 1] & 0x0f) << 8) )
#define NOISE_PERIOD(_psg)			( (_psg)->regs[AY_NOISEPER] & 0x1f)
#define TONE_VOLUME(_psg, _chan)	( (_psg)->regs[AY_AVOL + (_chan)] & 0x0f)
#define TONE_ENVELOPE(_psg, _chan)	(((_psg)->regs[AY_AVOL + (_chan)] >> 4) & 1)
#define ENVELOPE_PERIOD(_psg)		(((_psg)->regs[AY_EFINE] | ((_psg)->regs[AY_ECOARSE]<<8)))

/*************************************
 *
 *  Type definitions
 *
 *************************************/

#define AY8910_DEFAULT_LOADS		{1000, 1000, 1000}

typedef struct _ay_ym_param ay_ym_param;
struct _ay_ym_param
{
	double r_up;
	double r_down;
	int    res_count;
	double res[32];
};

struct AY8910
{
	int index;
	int streams;
	INT16 Channel;
	int ready;
	INT32 register_latch;
	UINT8 regs[16];
	INT32 last_enable;
	INT32 count[NUM_CHANNELS];
	UINT8 Output[NUM_CHANNELS];
	UINT8 output_noise;
	INT32 count_noise;
	INT32 count_env;
	INT8 CountEnv;
	UINT32 VolE;
	UINT8 Hold,Alternate,Attack,Holding;
	INT32 RNG;
	UINT8 env_step_mask;
	/* init parameters ... */
	int step;
	int zero_is_off;
	UINT8 vol_enabled[NUM_CHANNELS];
	ay_ym_param *par;
	ay_ym_param *par_env;
	INT32 VolTable[NUM_CHANNELS][16];
	INT32 VolTableE[NUM_CHANNELS][32];
	INT32 vol3d_tab[8*32*32*32];
	mem_read_handler PortAread;
	mem_read_handler PortBread;
	mem_write_handler PortAwrite;
	mem_write_handler PortBwrite;

};
static struct AY8910 AYpsg[MAX_8910];		/* array of psg's */

/*************************************
 *
 *  Static
 *
 *************************************/

static ay_ym_param ym2149_param =
{
	630, 801,
	16,
	{ 73770, 37586, 27458, 21451, 15864, 12371, 8922,  6796,
	   4763,  3521,  2403,  1737,  1123,   762,  438,   251 },
};

static ay_ym_param ym2149_paramE =
{
	630, 801,
	32,
	{ 103350, 73770, 52657, 37586, 32125, 27458, 24269, 21451,
	   18447, 15864, 14009, 12371, 10506,  8922,  7787,  6796,
	    5689,  4763,  4095,  3521,  2909,  2403,  2043,  1737,
	    1397,  1123,   925,   762,   578,   438,   332,   251 },
};

#if 0
/* RL = 1000, Hacker Kay normalized, 2.1V to 3.2V */
static const ay_ym_param ay8910_param =
{
	664, 913,
	16,
	{ 85785, 34227, 26986, 20398, 14886, 10588,  7810,  4856,
	   4120,  2512,  1737,  1335,  1005,   747,   586,    451 },
};
/*
 * RL = 3000, Hacker Kay normalized pattern, 1.5V to 2.8V
 * These values correspond with guesses based on Gyruss schematics
 * They work well with scramble as well.
 */
static ay_ym_param ay8910_param =
{
	930, 454,
	16,
	{ 85066, 34179, 27027, 20603, 15046, 10724, 7922, 4935,
	   4189,  2557,  1772,  1363,  1028,  766,   602,  464 },
};

/*
 * RL = 1000, Hacker Kay normalized pattern, 0.75V to 2.05V
 * These values correspond with guesses based on Gyruss schematics
 * They work well with scramble as well.
 */
static const ay_ym_param ay8910_param =
{
	1371, 313,
	16,
	{ 93399, 33289, 25808, 19285, 13940, 9846,  7237,  4493,
	   3814,  2337,  1629,  1263,   962,  727,   580,   458 },
};
#endif

/*
 * RL = 1000, Hacker Kay normalized pattern, 0.2V to 1.5V
 */
static const ay_ym_param ay8910_param =
{
	5806, 300,
	16,
	{ 118996, 42698, 33105, 24770, 17925, 12678,  9331,  5807,
        4936,  3038,  2129,  1658,  1271,   969,   781,   623 }
};
/*************************************
 *
 *  Inline
 *
 *************************************/


static INLINE void build_3D_table(double rl, ay_ym_param *par, ay_ym_param *par_env, int normalize, double factor, int zero_is_off, INT32 *tab)
{
	int j, j1, j2, j3, e, indx;
	double rt, rw, n;
	double min = 10.0,  max = 0.0;
	double *temp;

	temp = malloc(8*32*32*32*sizeof(*temp));

	for (e=0; e < 8; e++)
		for (j1=0; j1 < 32; j1++)
			for (j2=0; j2 < 32; j2++)
				for (j3=0; j3 < 32; j3++)
				{
					if (zero_is_off)
					{
						n  = (j1 != 0 || (e & 0x01)) ? 1 : 0;
						n += (j2 != 0 || (e & 0x02)) ? 1 : 0;
						n += (j3 != 0 || (e & 0x04)) ? 1 : 0;
					}
					else
						n = 3.0;

					rt = n / par->r_up + 3.0 / par->r_down + 1.0 / rl;
					rw = n / par->r_up;

					rw += 1.0 / ( (e & 0x01) ? par_env->res[j1] : par->res[j1]);
					rt += 1.0 / ( (e & 0x01) ? par_env->res[j1] : par->res[j1]);
					rw += 1.0 / ( (e & 0x02) ? par_env->res[j2] : par->res[j2]);
					rt += 1.0 / ( (e & 0x02) ? par_env->res[j2] : par->res[j2]);
					rw += 1.0 / ( (e & 0x04) ? par_env->res[j3] : par->res[j3]);
					rt += 1.0 / ( (e & 0x04) ? par_env->res[j3] : par->res[j3]);

					indx = (e << 15) | (j3<<10) | (j2<<5) | j1;
					temp[indx] = rw / rt;
					if (temp[indx] < min)
						min = temp[indx];
					if (temp[indx] > max)
						max = temp[indx];
				}

	if (normalize)
	{
		for (j=0; j < 32*32*32*8; j++)
			tab[j] = MAX_OUTPUT * (((temp[j] - min)/(max-min))) * factor;
	}
	else
	{
		for (j=0; j < 32*32*32*8; j++)
			tab[j] = MAX_OUTPUT * temp[j];
	}

	/* for (e=0;e<16;e++) printf("%d %d\n",e<<10, tab[e<<10]); */

	free(temp);
}

static INLINE void build_single_table(double rl, ay_ym_param *par, int normalize, INT32 *tab, int zero_is_off)
{
	int j;
	double rt, rw = 0;
	double temp[32], min=10.0, max=0.0;

	for (j=0; j < par->res_count; j++)
	{
		rt = 1.0 / par->r_down + 1.0 / rl;

		rw = 1.0 / par->res[j];
		rt += 1.0 / par->res[j];

		if (!(zero_is_off && j == 0))
		{
			rw += 1.0 / par->r_up;
			rt += 1.0 / par->r_up;
		}

		temp[j] = rw / rt;
		if (temp[j] < min)
			min = temp[j];
		if (temp[j] > max)
			max = temp[j];
	}
	if (normalize)
	{
		for (j=0; j < par->res_count; j++)
			tab[j] = MAX_OUTPUT * (((temp[j] - min)/(max-min)) - 0.25) * 0.5;
	}
	else
	{
		for (j=0; j < par->res_count; j++)
			tab[j] = MAX_OUTPUT * temp[j];
	}

}
static INLINE UINT16 mix_3D(struct AY8910 *psg)
{
	int indx = 0, chan;

	for (chan = 0; chan < NUM_CHANNELS; chan++)
		if (TONE_ENVELOPE(psg, chan))
		{
			indx |= (1 << (chan+15)) | ( psg->vol_enabled[chan] ? psg->VolE << (chan*5) : 0);
		}
		else
		{
			indx |= (psg->vol_enabled[chan] ? TONE_VOLUME(psg, chan) << (chan*5) : 0);
		}
	return psg->vol3d_tab[indx];
}

/*************************************
 *
 * Static functions
 *
 *************************************/

void _AYWriteReg(int n, int r, int v)
{

	struct AY8910 *psg = &AYpsg[n];


	psg->regs[r] = v;

	/* A note about the period of tones, noise and envelope: for speed reasons,*/
	/* we count down from the period to 0, but careful studies of the chip     */
	/* output prove that it instead counts up from 0 until the counter becomes */
	/* greater or equal to the period. This is an important difference when the*/
	/* program is rapidly changing the period to modulate the sound.           */
	/* To compensate for the difference, when the period is changed we adjust  */
	/* our internal counter.                                                   */
	/* Also, note that period = 0 is the same as period = 1. This is mentioned */
	/* in the YM2203 data sheets. However, this does NOT apply to the Envelope */
	/* period. In that case, period = 0 is half as period = 1. */
	switch( r )
	{
		case AY_AFINE:
		case AY_ACOARSE:
		case AY_BFINE:
		case AY_BCOARSE:
		case AY_CFINE:
		case AY_CCOARSE:
			/* No action required */
			break;
		case AY_NOISEPER:
			/* No action required */
			break;
		case AY_ENABLE:
			if ((psg->last_enable == -1) ||
			    ((psg->last_enable & 0x40) != (psg->regs[AY_ENABLE] & 0x40)))
			{
				/* write out 0xff if port set to input */

				if (psg->PortAwrite)
					(*psg->PortAwrite)( 0, (psg->regs[AY_ENABLE] & 0x40) ? psg->regs[AY_PORTA] : 0xff);
			}

			if ((psg->last_enable == -1) ||
			    ((psg->last_enable & 0x80) != (psg->regs[AY_ENABLE] & 0x80)))
			{
				/* write out 0xff if port set to input */
				if (psg->PortBwrite)
					(*psg->PortBwrite)(0, (psg->regs[AY_ENABLE] & 0x80) ? psg->regs[AY_PORTB] : 0xff);
			}

			psg->last_enable = psg->regs[AY_ENABLE];
			break;
		case AY_AVOL:
		case AY_BVOL:
		case AY_CVOL:
			/* No action required */
			break;
		case AY_EFINE:
		case AY_ECOARSE:
			/* No action required */
			break;
		case AY_ESHAPE:
			/* envelope shapes:
	        C AtAlH
	        0 0 x x  \___
	        0 1 x x  /___
	        1 0 0 0  \\\\
	        1 0 0 1  \___
	        1 0 1 0  \/\/
	        1 0 1 1  \¯¯¯
	        1 1 0 0  ////
	        1 1 0 1  /¯¯¯
	        1 1 1 0  /\/\
	        1 1 1 1  /___
	        The envelope counter on the AY-3-8910 has 16 steps. On the YM2149 it
	        has twice the steps, happening twice as fast.
	        */
			psg->Attack = (psg->regs[AY_ESHAPE] & 0x04) ? psg->env_step_mask : 0x00;
			if ((psg->regs[AY_ESHAPE] & 0x08) == 0)
			{
				/* if Continue = 0, map the shape to the equivalent one which has Continue = 1 */
				psg->Hold = 1;
				psg->Alternate = psg->Attack;
			}
			else
			{
				psg->Hold = psg->regs[AY_ESHAPE] & 0x01;
				psg->Alternate = psg->regs[AY_ESHAPE] & 0x02;
			}
			psg->CountEnv = psg->env_step_mask;
			psg->Holding = 0;
			psg->VolE = (psg->CountEnv ^ psg->Attack);
			break;
		case AY_PORTA:
			if (psg->regs[AY_ENABLE] & 0x40)
			{
				if (psg->PortAwrite)
					(*psg->PortAwrite)(0, psg->regs[AY_PORTA]);
				else
					logerror("warning - write %02x to 8910 #%d Port A\n",psg->regs[AY_PORTA],psg->index);
			}
			else
			{
				logerror("warning: write to 8910 #%d Port A set as input - ignored\n",psg->index);
			}
			break;
		case AY_PORTB:
			if (psg->regs[AY_ENABLE] & 0x80)
			{
				if (psg->PortBwrite)
					(*psg->PortBwrite)(0, psg->regs[AY_PORTB]);
				else
					logerror("warning - write %02x to 8910 #%d Port B\n",psg->regs[AY_PORTB],psg->index);
			}
			else
			{
				logerror("warning: write to 8910 #%d Port B set as input - ignored\n",psg->index);
			}
			break;
	}
}

static void AY8910_update(int chip, INT16 **buffer,int length)
{
	struct AY8910 *psg = &AYpsg[chip];
	INT16 *buf[NUM_CHANNELS];
	int chan;

	buf[0] = buffer[0];
	buf[1] = buffer[1];
	buf[2] = buffer[2];
	
	/* hack to prevent us from hanging when starting filtered outputs */
	if (!psg->ready)
	{
		for (chan = 0; chan < NUM_CHANNELS; chan++)
			if (buf[chan] != NULL)
				memset(buf[chan], 0, length * sizeof(*buf[chan]));
	}

	/* The 8910 has three outputs, each output is the mix of one of the three */
	/* tone generators and of the (single) noise generator. The two are mixed */
	/* BEFORE going into the DAC. The formula to mix each channel is: */
	/* (ToneOn | ToneDisable) & (NoiseOn | NoiseDisable). */
	/* Note that this means that if both tone and noise are disabled, the output */
	/* is 1, not 0, and can be modulated changing the volume. */

	/* buffering loop */
	while (length)
	{
		for (chan = 0; chan < NUM_CHANNELS; chan++)
		{
			psg->count[chan]++;
			if (psg->count[chan] >= TONE_PERIOD(psg, chan) * psg->step)
			{
				psg->Output[chan] ^= 1;
				psg->count[chan] = 0;;
			}
		}

		psg->count_noise++;
		if (psg->count_noise >= NOISE_PERIOD(psg) * psg->step)
		{
			/* Is noise output going to change? */
			if ((psg->RNG + 1) & 2)	/* (bit0^bit1)? */
			{
				psg->output_noise ^= 1;
			}

			/* The Random Number Generator of the 8910 is a 17-bit shift */
			/* register. The input to the shift register is bit0 XOR bit3 */
			/* (bit0 is the output). This was verified on AY-3-8910 and YM2149 chips. */

			/* The following is a fast way to compute bit17 = bit0^bit3. */
			/* Instead of doing all the logic operations, we only check */
			/* bit0, relying on the fact that after three shifts of the */
			/* register, what now is bit3 will become bit0, and will */
			/* invert, if necessary, bit14, which previously was bit17. */
			if (psg->RNG & 1)
				psg->RNG ^= 0x24000; /* This version is called the "Galois configuration". */
			psg->RNG >>= 1;
			psg->count_noise = 0;
		}

		for (chan = 0; chan < NUM_CHANNELS; chan++)
		{
			psg->vol_enabled[chan] = (psg->Output[chan] | TONE_ENABLEQ(psg, chan)) & (psg->output_noise | NOISE_ENABLEQ(psg, chan));
		}

		/* update envelope */
		if (psg->Holding == 0)
		{
			psg->count_env++;
			if (psg->count_env >= ENVELOPE_PERIOD(psg))
			{
				psg->count_env = 0;
				psg->CountEnv--;

				/* check envelope current position */
				if (psg->CountEnv < 0)
				{
					if (psg->Hold)
					{
						if (psg->Alternate)
							psg->Attack ^= psg->env_step_mask;
						psg->Holding = 1;
						psg->CountEnv = 0;
					}
					else
					{
						/* if CountEnv has looped an odd number of times (usually 1), */
						/* invert the output. */
						if (psg->Alternate && (psg->CountEnv & (psg->env_step_mask + 1)))
 							psg->Attack ^= psg->env_step_mask;

						psg->CountEnv &= psg->env_step_mask;
					}
				}

			}
		}
		psg->VolE = (psg->CountEnv ^ psg->Attack);

		if (psg->streams == NUM_CHANNELS)
		{
			for (chan = 0; chan < NUM_CHANNELS; chan++)
				if (TONE_ENVELOPE(psg,chan))
				{
					/* Envolope has no "off" state */
					*(buf[chan]++) = psg->VolTableE[chan][psg->vol_enabled[chan] ? psg->VolE : 0];
				}
				else
				{
					*(buf[chan]++) = psg->VolTable[chan][psg->vol_enabled[chan] ? TONE_VOLUME(psg, chan) : 0];
				}
		}
		else
		{
			*(buf[0]++) = mix_3D(psg);
			*(buf[1]++) =0;
			*(buf[2]++) = 0; // clear these buffers when not in use 
		}
		length--;
	}
}

/* write a register on AY8910 chip number 'n' */
void AYWriteReg(int chip, int r, int v)
{
	struct AY8910 *psg = &AYpsg[chip];


	if (r > 15) return;
	if (r < 14)
	{
		if (r == AY_ESHAPE || psg->regs[r] != v)
		{
			/* update the output buffer before changing the register */
			stream_update(psg->Channel,0);
		}
	}

	_AYWriteReg(chip,r,v);
}



unsigned char AYReadReg(int n, int r)
{
	struct AY8910 *psg = &AYpsg[n];


	if (r > 15) return 0;

	switch (r)
	{
	case AY_PORTA:
		if ((psg->regs[AY_ENABLE] & 0x40) != 0)
			log_cb(RETRO_LOG_DEBUG, LOGPRE "warning: read from 8910 #%d Port A set as output\n",n);
		/*
		   even if the port is set as output, we still need to return the external
		   data. Some games, like kidniki, need this to work.
		 */
		if (psg->PortAread) psg->regs[AY_PORTA] = (*psg->PortAread)(0);
		else log_cb(RETRO_LOG_DEBUG, LOGPRE "PC %04x: warning - read 8910 #%d Port A\n",activecpu_get_pc(),n);
		break;
	case AY_PORTB:
		if ((psg->regs[AY_ENABLE] & 0x80) != 0)
			log_cb(RETRO_LOG_DEBUG, LOGPRE "warning: read from 8910 #%d Port B set as output\n",n);
		if (psg->PortBread) psg->regs[AY_PORTB] = (*psg->PortBread)(0);
		else log_cb(RETRO_LOG_DEBUG, LOGPRE "PC %04x: warning - read 8910 #%d Port B\n",activecpu_get_pc(),n);
		break;
	}
	return psg->regs[r];
}


void AY8910Write(int chip,int a,int data)
{
	struct AY8910 *psg = &AYpsg[chip];

	if (a & 1)
	{	/* Data port */
		AYWriteReg(chip,psg->register_latch,data);
	}
	else
	{	/* Register port */
		psg->register_latch = data & 0x0f;
	}
}

int AY8910Read(int chip)
{
	struct AY8910 *psg = &AYpsg[chip];

	return AYReadReg(chip,psg->register_latch);
}


static void build_mixer_table(int chip, int type)
{

	int	normalize = 0;
	int	chan;
	struct AY8910 *psg = &AYpsg[chip];

/*	if ((psg->intf->flags & AY8910_LEGACY_OUTPUT) != 0)
	{
		logerror("AY-3-8910/YM2149 "using legacy output levels!\n");
		normalize = 1;
	}
*/
	//if (type == 0)
	if (1)
	{
		normalize = 1;
		psg->step = 1;
		psg->par = &ay8910_param;
		psg->par_env = &ay8910_param;
		psg->zero_is_off = 1;
		psg->streams= 3;
	}

	else if ( type == 1)
	{	
			normalize = 1;
			psg->step = 2;
			psg->par = &ym2149_param;
			psg->par_env = &ym2149_paramE;
			psg->zero_is_off = 0;
			psg->streams= 1;
	}
	psg->env_step_mask = psg->step * 16 - 1;
	
	for (chan=0; chan < NUM_CHANNELS; chan++)
	{
		build_single_table(1.0, psg->par, normalize, psg->VolTable[chan], psg->zero_is_off);
		build_single_table(1.0, psg->par_env, normalize, psg->VolTableE[chan], 0);
	}
	build_3D_table(1.0, psg->par, psg->par_env, normalize, 3, psg->zero_is_off, psg->vol3d_tab);
}

/*************************************
 *
 * Public functions
 *
 *   used by e.g. YM2203, YM2210 ...
 *
 *************************************/

void AY8910_set_clock(int chip,int clock)
{

}


void AY8910_set_volume(int chip,int channel,int volume)
{
	struct AY8910 *psg = &AYpsg[chip];
	int ch;

	for (ch = 0; ch < 3; ch++)
		if (channel == ch || channel == ALL_8910_CHANNELS)
			mixer_set_volume(psg->Channel + ch, volume);
}










void AY8910_reset(int chip)
{
	int i;
	struct AY8910 *psg = &AYpsg[chip];

	psg->register_latch = 0;
	psg->RNG = 1;
	psg->Output[0] = 0;
	psg->Output[1] = 0;
	psg->Output[2] = 0;
	psg->count[0] = 0;
	psg->count[1] = 0;
	psg->count[2] = 0;
	psg->count_noise = 0;
	psg->count_env = 0;
	psg->output_noise = 0x01;
	psg->last_enable = -1;	/* force a write */
	for (i = 0;i < AY_PORTA;i++)
		_AYWriteReg(chip,i,0);	/* AYWriteReg() uses the timer system; we cannot */
								/* call it at this time because the timer system */
								/* has not been initialized. */
psg->ready = 1;
}

void AY8910_sh_reset(void)
{
	int i;

	for (i = 0;i < num + ym_num;i++)
		AY8910_reset(i);
}

static int AY8910_init(const char *chip_name,int chip,
		int clock,int volume,int sample_rate,
		mem_read_handler portAread,mem_read_handler portBread,
		mem_write_handler portAwrite,mem_write_handler portBwrite)
{
	int i;
	struct AY8910 *psg = &AYpsg[chip];
	char buf[3][40];
	const char *name[3];
	int vol[3];


/* causes crashes with YM2610 games - overflow?*/
/*	if (options.use_filter)*/
	sample_rate = (clock/8) ;

	memset(psg,0,sizeof(struct AY8910));
	psg->PortAread = portAread;
	psg->PortBread = portBread;
	psg->PortAwrite = portAwrite;
	psg->PortBwrite = portBwrite;
	for (i = 0;i < 3;i++)
	{
		vol[i] = volume;
		name[i] = buf[i];
		sprintf(buf[i],"%s #%d Ch %c",chip_name,chip,'A'+i);
	}
	psg->Channel = stream_init_multi(3,name,vol,sample_rate,chip,AY8910_update);

	if (psg->Channel == -1)
		return 1;

	//AY8910_set_clock(chip,clock);

	return 0;
}


int AY8910_sh_start(const struct MachineSound *msound)
{
	int chip;
	const struct AY8910interface *intf = msound->sound_interface;

	num = intf->num;

	for (chip = 0;chip < num;chip++)
	{
		if (AY8910_init(sound_name(msound),chip+ym_num,intf->baseclock,
				intf->mixing_level[chip] & 0xffff,
				Machine->sample_rate,
				intf->portAread[chip],intf->portBread[chip],
				intf->portAwrite[chip],intf->portBwrite[chip]) != 0)
			return 1;
		build_mixer_table(chip+ym_num,0);
	}
	return 0;
}

void AY8910_sh_stop(void)
{
	num = 0;
}

int AY8910_sh_start_ym(const struct MachineSound *msound)
{
	int chip;
	const struct AY8910interface *intf = msound->sound_interface;
	ym_num = intf->num;
	ay8910_index_ym = num;

	for (chip = 0;chip < ym_num;chip++)
	{
		if (AY8910_init(sound_name(msound),chip+num,intf->baseclock,
				intf->mixing_level[chip] & 0xffff,
				Machine->sample_rate,
				intf->portAread[chip],intf->portBread[chip],
				intf->portAwrite[chip],intf->portBwrite[chip]) != 0)
			return 1;
		build_mixer_table(chip+num,1);
	}
	return 0;
}

void AY8910_sh_stop_ym(void)
{
	ym_num = 0;
}
/*************************************
 *
 * Read/Write Handlers
 *
 *************************************/

READ_HANDLER( AY8910_read_port_0_r ) { return AY8910Read(0); }
READ_HANDLER( AY8910_read_port_1_r ) { return AY8910Read(1); }
READ_HANDLER( AY8910_read_port_2_r ) { return AY8910Read(2); }
READ_HANDLER( AY8910_read_port_3_r ) { return AY8910Read(3); }
READ_HANDLER( AY8910_read_port_4_r ) { return AY8910Read(4); }
READ16_HANDLER( AY8910_read_port_0_lsb_r ) { return AY8910Read(0); }
READ16_HANDLER( AY8910_read_port_1_lsb_r ) { return AY8910Read(1); }
READ16_HANDLER( AY8910_read_port_2_lsb_r ) { return AY8910Read(2); }
READ16_HANDLER( AY8910_read_port_3_lsb_r ) { return AY8910Read(3); }
READ16_HANDLER( AY8910_read_port_4_lsb_r ) { return AY8910Read(4); }
READ16_HANDLER( AY8910_read_port_0_msb_r ) { return AY8910Read(0) << 8; }
READ16_HANDLER( AY8910_read_port_1_msb_r ) { return AY8910Read(1) << 8; }
READ16_HANDLER( AY8910_read_port_2_msb_r ) { return AY8910Read(2) << 8; }
READ16_HANDLER( AY8910_read_port_3_msb_r ) { return AY8910Read(3) << 8; }
READ16_HANDLER( AY8910_read_port_4_msb_r ) { return AY8910Read(4) << 8; }

WRITE_HANDLER( AY8910_control_port_0_w ) { AY8910Write(0,0,data); }
WRITE_HANDLER( AY8910_control_port_1_w ) { AY8910Write(1,0,data); }
WRITE_HANDLER( AY8910_control_port_2_w ) { AY8910Write(2,0,data); }
WRITE_HANDLER( AY8910_control_port_3_w ) { AY8910Write(3,0,data); }
WRITE_HANDLER( AY8910_control_port_4_w ) { AY8910Write(4,0,data); }
WRITE16_HANDLER( AY8910_control_port_0_lsb_w ) { if (ACCESSING_LSB) AY8910Write(0,0,data & 0xff); }
WRITE16_HANDLER( AY8910_control_port_1_lsb_w ) { if (ACCESSING_LSB) AY8910Write(1,0,data & 0xff); }
WRITE16_HANDLER( AY8910_control_port_2_lsb_w ) { if (ACCESSING_LSB) AY8910Write(2,0,data & 0xff); }
WRITE16_HANDLER( AY8910_control_port_3_lsb_w ) { if (ACCESSING_LSB) AY8910Write(3,0,data & 0xff); }
WRITE16_HANDLER( AY8910_control_port_4_lsb_w ) { if (ACCESSING_LSB) AY8910Write(4,0,data & 0xff); }
WRITE16_HANDLER( AY8910_control_port_0_msb_w ) { if (ACCESSING_MSB) AY8910Write(0,0,data >> 8); }
WRITE16_HANDLER( AY8910_control_port_1_msb_w ) { if (ACCESSING_MSB) AY8910Write(1,0,data >> 8); }
WRITE16_HANDLER( AY8910_control_port_2_msb_w ) { if (ACCESSING_MSB) AY8910Write(2,0,data >> 8); }
WRITE16_HANDLER( AY8910_control_port_3_msb_w ) { if (ACCESSING_MSB) AY8910Write(3,0,data >> 8); }
WRITE16_HANDLER( AY8910_control_port_4_msb_w ) { if (ACCESSING_MSB) AY8910Write(4,0,data >> 8); }

WRITE_HANDLER( AY8910_write_port_0_w ) { AY8910Write(0,1,data); }
WRITE_HANDLER( AY8910_write_port_1_w ) { AY8910Write(1,1,data); }
WRITE_HANDLER( AY8910_write_port_2_w ) { AY8910Write(2,1,data); }
WRITE_HANDLER( AY8910_write_port_3_w ) { AY8910Write(3,1,data); }
WRITE_HANDLER( AY8910_write_port_4_w ) { AY8910Write(4,1,data); }
WRITE16_HANDLER( AY8910_write_port_0_lsb_w ) { if (ACCESSING_LSB) AY8910Write(0,1,data & 0xff); }
WRITE16_HANDLER( AY8910_write_port_1_lsb_w ) { if (ACCESSING_LSB) AY8910Write(1,1,data & 0xff); }
WRITE16_HANDLER( AY8910_write_port_2_lsb_w ) { if (ACCESSING_LSB) AY8910Write(2,1,data & 0xff); }
WRITE16_HANDLER( AY8910_write_port_3_lsb_w ) { if (ACCESSING_LSB) AY8910Write(3,1,data & 0xff); }
WRITE16_HANDLER( AY8910_write_port_4_lsb_w ) { if (ACCESSING_LSB) AY8910Write(4,1,data & 0xff); }
WRITE16_HANDLER( AY8910_write_port_0_msb_w ) { if (ACCESSING_MSB) AY8910Write(0,1,data >> 8); }
WRITE16_HANDLER( AY8910_write_port_1_msb_w ) { if (ACCESSING_MSB) AY8910Write(1,1,data >> 8); }
WRITE16_HANDLER( AY8910_write_port_2_msb_w ) { if (ACCESSING_MSB) AY8910Write(2,1,data >> 8); }
WRITE16_HANDLER( AY8910_write_port_3_msb_w ) { if (ACCESSING_MSB) AY8910Write(3,1,data >> 8); }
WRITE16_HANDLER( AY8910_write_port_4_msb_w ) { if (ACCESSING_MSB) AY8910Write(4,1,data >> 8); }


