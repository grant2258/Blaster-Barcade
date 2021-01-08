/***********************************************************************************************

    Sega System C/C2 Driver
    driver by David Haywood and Aaron Giles
    ---------------------------------------
    Last Update 17 Sept 2005


    Sega's C2 was used between 1989 and 1994, the hardware being very similar to that
    used by the Sega MegaDrive/Genesis Home Console Sega produced around the same time.

    Year  Game                  Developer         Versions Dumped  Board  Status        Gen Ver Exists?
    ====  ====================  ================  ===============  =====  ============  ===============
    1989  Bloxeed               Sega / Elorg      Eng              C      Playable      n
    1990  Columns               Sega              Jpn              C      Playable      y
    1990  Columns II            Sega              Jpn              C      Playable      n
    1990  Borench               Sega              Eng              C2     Playable      n
    1990  ThunderForce AC       Sega / Technosoft Eng, Jpn, EngBL  C2     Playable      y (as ThunderForce 3?)
    1991  Twin Squash           Sega              Eng              C2     Playable      n
    1992  Ribbit!               Sega              Eng              C2     Playable      ?
    1992  Tant-R                Sega              Jpn, JpnBL       C2     Playable      y
    1992  Puyo Puyo             Sega / Compile    Jpn (2 Vers)     C2     Playable      y
    1994  Ichidant-R            Sega              Jpn              C2     Playable      y
    1994  PotoPoto              Sega              Jpn              C2     Playable      n
    1994  Puyo Puyo 2           Compile           Jpn              C2     Playable      y
    1994  Stack Columns         Sega              Jpn              C2     Playable      n
    1994  Zunzunkyou No Yabou   Sega              Jpn              C2     Playable      n

        + Print Club Vols 1,2,4,5 (and 3?)


    Notes:  Eng indicates game is in the English Language, Most Likely a European / US Romset
            Jpn indicates the game plays in Japanese and is therefore likely a Japanes Romset

            Bloxeed doesn't Read from the Protection Chip at all; all of the other games do.
            Currently the protection chip is mostly understood, and needs a table of 256
            4-bit values for each game. In all cases except for Poto Poto and Puyo Puyo 2,
            the table is embedded in the code. Workarounds for the other 2 cases are
            provided.

            I'm assuming System-C was the Board without the uPD7759 chip and System-C2 was the
            version of the board with it, this could be completely wrong but it doesn't really
            matter anyway.

    Bugs:   Puyo Puyo ends up with a black screen after doing memory tests
            Battery-backed RAM needs to be figured out

    Thanks: (in no particular order) to any MameDev that helped me out .. (OG, Mish etc.)
            Charles MacDonald for his C2Emu .. without it working out what were bugs in my code
                and issues due to protection would have probably killed the driver long ago :p
            Razoola & Antiriad .. for helping teach me some 68k ASM needed to work out just why
                the games were crashing :)
            Sega for producing some Fantastic Games...
            and anyone else who knows they've contributed :)

************************************************************************************************

    Hiscores:

    Bloxeed  @ f400-????            [key = ???]
    Columns  @ fc00-ffff            [key = '(C) SEGA 1990.JAN BY.TAKOSUKEZOU' @ fc00,ffe0]
    Columns2 @ fc00-ffff            [key = '(C) SEGA 1990.SEP.COLUMNS2 JAPAN' @ fc00,fd00,fe00,ffe0]
    Borench  @ f400-f5ff            [key = 'EIJI' in last word]
    TForceAC @ 8100-817f/8180-81ff  [key = '(c)Tehcno soft90' @ 8070 and 80f0]
    TantR    @ fc00-fcff/fd00-fdff  [key = 0xd483 in last word]
    PuyoPuyo @ fc00-fdff/fe00-ffff  [key = 0x28e1 in first word]
    Ichidant @ fc00-fcff/fd00-fdff  [key = 0x85a9 in last word]
    StkClmns @ fc00-fc7f/fc80-fcff  [key = ???]
    PuyoPuy2
    PotoPoto
    ZunkYou

***********************************************************************************************/


#include "driver.h"
#include "vidhrdw/generic.h"
#include "cpu/m68000/m68000.h"
#include "state.h"
#include "machine/random.h"
#include "genesis.h"

#define XL1_CLOCK			640000
#define XL2_CLOCK			53693175


#define LOG_PROTECTION		0
#define LOG_PALETTE			0
#define LOG_IOCHIP			0


/******************************************************************************
    Global variables
******************************************************************************/

/* interrupt states */
static UINT8		irq2_int;			/* INT2 */
static UINT8		scanline_int;		/* INT4 - programmable */
static UINT8		vblank_int;			/* INT6 - on every VBLANK */

static mame_timer *	scan_timer;

/* internal states */
static UINT8 		misc_io_data[0x10];	/* holds values written to the I/O chip */

/* protection-related tracking */
static const UINT32 *prot_table;		/* table of protection values */
static UINT8 		prot_write_buf;		/* remembers what was written */
static UINT8		prot_read_buf;		/* remembers what was returned */

/* palette-related variables */
static UINT8		alt_palette_mode;
static UINT8		palbank;
static UINT8		bg_palbase;
static UINT8		sp_palbase;

/* sound-related variables */
static UINT8		sound_banks;		/* number of sound banks */
static UINT8		bloxeed_sound;		/* use kludge for bloxeed sound? */


/******************************************************************************
    Interrupt handling
*******************************************************************************

    The C/C2 System uses 3 Different Interrupts, IRQ2, IRQ4 and IRQ6.

    IRQ6 = Vblank, this happens after the last visible line of the display has
            been drawn (after line 224)

    IRQ4 = H-Int, this happens based upon the value in H-Int Counter.  If the
            Horizontal Interrupt is enabled and the Counter Value = 0 there
            will be a Level 4 Interrupt Triggered

    IRQ2 = sound int, generated by the YM3438

    --------

    More H-Counter Information:

    Providing Horizontal Interrupts are active the H-Counter will be loaded
    with the value stored in register #10 (0x0A) at the following times:
        (1) At the top of the display, before drawing the first line
        (2) When the counter has expired
        (3) During the VBlank Period (lines 224-261)
    The Counter is decreased by 1 after every line.

******************************************************************************/

/* call this whenever the interrupt state has changed */
static void update_interrupts(void)
{
	int level = 0;

	/* determine which interrupt is active */
	if (irq2_int) level = 2;
	if (scanline_int) level = 4;
	if (vblank_int) level = 6;

	/* either set or clear the appropriate lines */
	if (level)
		cpu_set_irq_line(0, level, ASSERT_LINE);
	else
		cpu_set_irq_line(0, 7, CLEAR_LINE);
}


/* timer callback to turn off the IRQ4 signal after a short while */
static void vdp_int4_off(int param)
{
	scanline_int = 0;
	update_interrupts();
}


/* timer callback to handle reloading the H counter and generate IRQ4 */
void vdp_reload_counter(int scanline)
{
	/* generate an int if they're enabled */
	if (genesis_vdp_regs[0] & 0x10)/* && !(misc_io_data[7] & 0x10))*/
		if (scanline != 0 || genesis_vdp_regs[10] == 0)
		{
			scanline_int = 1;
			update_interrupts();
			timer_set(cpu_getscanlinetime(scanline + 1), 0, vdp_int4_off);
		}

	/* advance to the next scanline */
	/* behavior 2: 0 count means interrupt after one scanline */
	/* (this behavior matches the Sega C2 emulator) */
	scanline += genesis_vdp_regs[10] + 1;
	if (scanline >= 224)
		scanline = 0;

	/* set a timer */
	timer_adjust(scan_timer, cpu_getscanlinetime(scanline) + cpu_getscanlineperiod() * (320. / 342.), scanline, 0);
}


/* timer callback to turn off the IRQ6 signal after a short while */
static void vdp_int6_off(int param)
{
	vblank_int = 0;
	update_interrupts();
}


/* interrupt callback to generate the VBLANK interrupt */
INTERRUPT_GEN( genesis_vblank_interrupt )
{
	/* generate the interrupt */
	vblank_int = 1;
	update_interrupts();

	/* set a timer to turn it off */
	timer_set(cpu_getscanlineperiod() * (22. / 342.), 0, vdp_int6_off);
}


/* interrupt callback to generate the YM3438 interrupt */
void genesis_irq2_interrupt(int state)
{
	irq2_int = state;
	update_interrupts();
}


/******************************************************************************
    Machine init
*******************************************************************************

    This is called at init time, when it's safe to create a timer. We use
    it to prime the scanline interrupt timer.

******************************************************************************/

MACHINE_INIT( segac2new )
{

	/* set the first scanline 0 timer to go off */
	scan_timer = timer_alloc(vdp_reload_counter);
	timer_adjust(scan_timer, cpu_getscanlinetime(0) + cpu_getscanlineperiod() * (320. / 342.), 0, 0);


	/* determine how many sound banks */
	sound_banks = 0;
	if (memory_region(REGION_SOUND1))
		sound_banks = memory_region_length(REGION_SOUND1) / 0x20000;

	/* reset the protection */
	prot_write_buf = 0;
	prot_read_buf = 0;
	alt_palette_mode = 0;

	palbank = 0;
	bg_palbase = 0;
	sp_palbase = 0;
}


/******************************************************************************
    Sound handlers
*******************************************************************************

    These handlers are responsible for communicating with the (genenerally)
    8-bit sound chips. All accesses are via the low byte.

    The Sega C/C2 system uses a YM3438 (compatible with the YM2612) for FM-
    based music generation, and an SN76489 for PSG and noise effects. The
    C2 board also appears to have a UPD7759 for sample playback.

******************************************************************************/

/* handle reads from the YM3438 */
static READ16_HANDLER( ym3438_r )
{
	switch (offset)
	{
		case 0: return YM2612_status_port_0_A_r(0);
		case 1: return YM2612_read_port_0_r(0);
		case 2: return YM2612_status_port_0_B_r(0);
	}
	return 0xff;
}

/* handle writes to the YM3438 */
static WRITE16_HANDLER( ym3438_w )
{
	/* only works if we're accessing the low byte */
	if (ACCESSING_LSB)
	{
		static UINT8 last_port;

		/* kludge for Bloxeed - it seems to accidentally trip timer 2  */
		/* and has no recourse for clearing the interrupt; until we    */
		/* find more documentation on the 2612/3438, it's unknown what */
		/* to do here */
		if (bloxeed_sound && last_port == 0x27 && (offset & 1))
			data &= ~0x08;

		switch (offset)
		{
			case 0: YM2612_control_port_0_A_w(0, data & 0xff);	last_port = data;	break;
			case 1: YM2612_data_port_0_A_w(0, data & 0xff);							break;
			case 2: YM2612_control_port_0_B_w(0, data & 0xff);	last_port = data;	break;
			case 3: YM2612_data_port_0_B_w(0, data & 0xff);							break;
		}
	}
}


/* handle writes to the UPD7759 */
static WRITE16_HANDLER( segac2new_upd7759_w )
{
	/* make sure we have a UPD chip */
	if (!sound_banks)
		return;

	/* only works if we're accessing the low byte */
	if (ACCESSING_LSB)
	{
		UPD7759_reset_w(0, 0);
		UPD7759_reset_w(0, 1);
		UPD7759_port_w(0, data & 0xff);
		UPD7759_start_w(0, 0);
		UPD7759_start_w(0, 1);
	}
}


/* handle writes to the SN764896 */
WRITE16_HANDLER( sn76489_w )
{
	/* only works if we're accessing the low byte */
	if (ACCESSING_LSB)
		SN76496_0_w(0, data & 0xff);
}



/******************************************************************************
    Palette RAM Read / Write Handlers
*******************************************************************************

    The following Read / Write Handlers are used when accessing Palette RAM.
    The C2 Hardware appears to use 4 Banks of Colours 1 of which can be Mapped
    to 0x8C0000 - 0x8C03FF at any given time by writes to 0x84000E (This same
    address also looks to be used for things like Sample Banking)

    Each Colour uses 15-bits (from a 16-bit word) in the Format
        xBGRBBBB GGGGRRRR  (x = unused, B = Blue, G = Green, R = Red)

    As this works out the Palette RAM Stores 2048 from a Possible 4096 Colours
    at any given time.

******************************************************************************/

/* handle reads from the paletteram */
static READ16_HANDLER( palette_r )
{
	offset &= 0x1ff;
	if (alt_palette_mode)
		offset = ((offset << 1) & 0x100) | ((offset << 2) & 0x80) | ((~offset >> 2) & 0x40) | ((offset >> 1) & 0x20) | (offset & 0x1f);
	return paletteram16[offset + palbank * 0x200];
}


/* handle writes to the paletteram */
static WRITE16_HANDLER( palette_w )
{
	int r,g,b,newword;

	/* adjust for the palette bank */
	offset &= 0x1ff;
	if (alt_palette_mode)
		offset = ((offset << 1) & 0x100) | ((offset << 2) & 0x80) | ((~offset >> 2) & 0x40) | ((offset >> 1) & 0x20) | (offset & 0x1f);
	offset += palbank * 0x200;

	/* combine data */
	COMBINE_DATA(&paletteram16[offset]);
	newword = paletteram16[offset];

	/* up to 8 bits */
	r = ((newword << 4) & 0xf0) | ((newword >>  9) & 0x08);
	g = ((newword >> 0) & 0xf0) | ((newword >> 10) & 0x08);
	b = ((newword >> 4) & 0xf0) | ((newword >> 11) & 0x08);
	r |= r >> 5;
	g |= g >> 5;
	b |= b >> 5;

	/* set the color */
	palette_set_color(offset, r, g, b);
}



/******************************************************************************
    Palette Tables
*******************************************************************************

    Both the background and sprites within the VDP have 4 possible palettes
    to select from. External hardware on the C2 boards, controlled by the
    EPM5032 chip and the I/O chip, allows for more complex palette selection.
    The actual palette entry comes from:

        Bits 10-9 = output from I/O port H
        Bits  8-5 = output from EPM5032
        Bits  4-0 = direct from the VDP

    In order to compute bits 8-5, the EPM5032 gets the sprite/background
    output line along with the two bits of palette info. For most games, the
    two bits of palette info map straight through as follows:

        Bits 10-9 = output from I/O port H
        Bits    8 = sprite/background select
        Bits  7-6 = palette selected by writing to prot_w
        Bits  5-4 = direct from the VDP palette select
        Bits  3-0 = direct from the VDP

    However, because there are 4 bits completely controlled by the EPM5032,
    it doesn't have to always map this way. An alternate palette mode can
    be selected which alters the output palette by swizzling the color
    RAM address bits.

******************************************************************************/

static void recompute_palette_tables(void)
{
	int i;

	for (i = 0; i < 4; i++)
	{
		int bgpal = 0x000 + bg_palbase * 0x40 + i * 0x10;
		int sppal = 0x100 + sp_palbase * 0x40 + i * 0x10;

		if (!alt_palette_mode)
		{
			genesis_bg_pal_lookup[i] = palbank * 0x200 + bgpal;
			genesis_sp_pal_lookup[i] = palbank * 0x200 + sppal;
		}
		else
		{
			genesis_bg_pal_lookup[i] = palbank * 0x200 + ((bgpal << 1) & 0x180) + ((~bgpal >> 2) & 0x40) + (bgpal & 0x30);
			genesis_sp_pal_lookup[i] = palbank * 0x200 + ((~sppal << 2) & 0x100) + ((sppal << 2) & 0x80) + ((~sppal >> 2) & 0x40) + ((sppal >> 2) & 0x20) + (sppal & 0x10);
		}
	}
}



/******************************************************************************
    I/O Read & Write Handlers
*******************************************************************************

    Controls, and Poto Poto reads 'S' 'E' 'G' and 'A' (SEGA) from this area
    as a form of protection.

    Lots of unknown writes however offset 0E certainly seems to be banking,
    both colours and sound sample banks.

******************************************************************************/

static READ16_HANDLER( io_chip_r )
{
	offset &= 0x1f/2;

	switch (offset)
	{
		/* I/O ports */
		case 0x00/2:
		case 0x02/2:
		case 0x04/2:
		case 0x06/2:
		case 0x08/2:
		case 0x0a/2:
		case 0x0c/2:
		case 0x0e/2:
			/* if the port is configured as an output, return the last thing written */
			if (misc_io_data[0x1e/2] & (1 << offset))
				return misc_io_data[offset];

			/* otherwise, return an input port */
			if (offset == 0x04/2 && sound_banks)
				return (readinputport(offset) & 0xbf) | (UPD7759_0_busy_r(0) << 6);
			return readinputport(offset);

		/* 'SEGA' protection */
		case 0x10/2:
			return 'S';
		case 0x12/2:
			return 'E';
		case 0x14/2:
			return 'G';
		case 0x16/2:
			return 'A';

		/* CNT register & mirror */
		case 0x18/2:
		case 0x1c/2:
			return misc_io_data[0x1c/2];

		/* port direction register & mirror */
		case 0x1a/2:
		case 0x1e/2:
			return misc_io_data[0x1e/2];
	}
	return 0xffff;
}


static WRITE16_HANDLER( io_chip_w )
{
	UINT8 newbank;
	UINT8 old;

	/* generic implementation */
	offset &= 0x1f/2;
	old = misc_io_data[offset];
	misc_io_data[offset] = data;

	switch (offset)
	{
		/* I/O ports */
		case 0x00/2:
		case 0x02/2:
		case 0x04/2:
		case 0x08/2:
		case 0x0a/2:
		case 0x0c/2:
			break;

		/* miscellaneous output */
		case 0x06/2:
			/*
             D7 : To pin 3 of JP15. (Watchdog clock control)
             D6 : To MUTE input pin on TDA1518BQ amplifier.
             D5 : To CN2 pin 10. (Unknown purpose)
             D4 : To CN2 pin 11. (Unknown purpose)
             D3 : To CN1 pin K. (Coin lockout 2)
             D2 : To CN1 pin 9. (Coin lockout 1)
             D1 : To CN1 pin J. (Coin meter 2)
             D0 : To CN1 pin 8. (Coin meter 1)
            */
/*          coin_lockout_w(1, data & 0x08);
            coin_lockout_w(0, data & 0x04); */
			coin_counter_w(1, data & 0x02);
			coin_counter_w(0, data & 0x01);
			break;

		/* banking */
		case 0x0e/2:
			/*
             D7 : To pin A19 of CN4
             D6 : To pin B19 of CN4
             D5 : ?
             D4 : ?
             D3 : To pin 31 of uPD7759 sample ROM (A18 on a 27C040)
             D2 : To pin 30 of uPD7759 sample ROM (A17 on a 27C040)
             D1 : To A10 of color RAM
             D0 : To A9 of color RAM
            */
			newbank = data & 3;
			if (newbank != palbank)
			{
				force_partial_update(cpu_getscanline() + 1);
				palbank = newbank;
				recompute_palette_tables();
			}
			if (sound_banks > 1)
			{
				newbank = (data >> 2) & (sound_banks - 1);
				UPD7759_set_bank_base(0, newbank * 0x20000);
			}
			break;

		/* CNT register */
		case 0x1c/2:
			break;
	}
}



/******************************************************************************
    Control Write Handler
*******************************************************************************

    Seems to control some global states. The most important bit is the low
    one, which enables/disables the display. This is used while tiles are
    being modified in Bloxeed.

******************************************************************************/

static WRITE16_HANDLER( control_w )
{
	/* skip if not LSB */
	if (!ACCESSING_LSB)
		return;
	data &= 0x0f;

	/* bit 0 controls display enable */
	genesis_enable_display(~data & 1);

	/* bit 1 resets the protection */
	if (!(data & 2))
		prot_write_buf = prot_read_buf = 0;

	/* bit 2 controls palette shuffling; only ribbit and twinsqua use this feature */
	alt_palette_mode = ((~data & 4) >> 2);
	recompute_palette_tables();
}



/******************************************************************************
    Protection Read / Write Handlers
*******************************************************************************

    The protection chip is fairly simple. Writes to it control the palette
    banking for the sprites and backgrounds. The low 4 bits are also
    remembered in a 2-stage FIFO buffer. A read from this chip should
    return a value from a 256x4-bit table. The index into this table is
    computed by taking the second-to-last value written in the upper 4 bits,
    and the previously-fetched table value in the lower 4 bits.

******************************************************************************/

/* protection chip reads */
static READ16_HANDLER( prot_r )
{
	if (LOG_PROTECTION) logerror("%06X:protection r=%02X\n", activecpu_get_previouspc(), prot_table ? prot_read_buf : 0xff);
	return prot_read_buf | 0xf0;
}


/* protection chip writes */
static WRITE16_HANDLER( prot_w )
{
	int new_sp_palbase = (data >> 2) & 3;
	int new_bg_palbase = data & 3;
	int table_index;

	/* only works for the LSB */
	if (!ACCESSING_LSB)
		return;

	/* compute the table index */
	table_index = (prot_write_buf << 4) | prot_read_buf;

	/* keep track of the last write for the next table lookup */
	prot_write_buf = data & 0x0f;

	/* determine the value to return, should a read occur */
	if (prot_table)
		prot_read_buf = (prot_table[table_index >> 3] << (4 * (table_index & 7))) >> 28;
	if (LOG_PROTECTION) logerror("%06X:protection w=%02X, new result=%02X\n", activecpu_get_previouspc(), data & 0x0f, prot_read_buf);

	/* if the palette changed, force an update */
	if (new_sp_palbase != sp_palbase || new_bg_palbase != bg_palbase)
	{
		force_partial_update(cpu_getscanline() + 1);
		sp_palbase = new_sp_palbase;
		bg_palbase = new_bg_palbase;
		recompute_palette_tables();
		if (LOG_PALETTE) logerror("Set palbank: %d/%d (scan=%d)\n", bg_palbase, sp_palbase, cpu_getscanline());
	}
}



/******************************************************************************
    Counter/timer I/O
*******************************************************************************

    There appears to be a chip that is used to count coins and track time
    played, or at the very least the current status of the game. All games
    except Puyo Puyo 2 and Poto Poto access this in a mostly consistent
    manner.

******************************************************************************/

static WRITE16_HANDLER( counter_timer_w )
{
	/* only LSB matters */
	if (ACCESSING_LSB)
	{
		/*int value = data & 1;*/
		switch (data & 0x1e)
		{
			case 0x00:	/* player 1 start/stop */
			case 0x02:	/* player 2 start/stop */
			case 0x04:	/* ??? */
			case 0x06:	/* ??? */
			case 0x08:	/* player 1 game timer? */
			case 0x0a:	/* player 2 game timer? */
			case 0x0c:	/* ??? */
			case 0x0e:	/* ??? */
				break;

			case 0x10:	/* coin counter */
              coin_counter_w(0,1);
              coin_counter_w(0,0);
				break;

			case 0x12:	/* set coinage info -- followed by two 4-bit values */
				break;

			case 0x14:	/* game timer? (see Tant-R) */
			case 0x16:	/* intro timer? (see Tant-R) */
			case 0x18:	/* ??? */
			case 0x1a:	/* ??? */
			case 0x1c:	/* ??? */
				break;

			case 0x1e:	/* reset */
				break;
		}
	}
}



/******************************************************************************
    NVRAM Handling
*******************************************************************************

    There is a battery on the board that keeps the high scores. However,
    simply saving the RAM doesn't seem to be good enough. This is still
    pending investigation.

******************************************************************************/

/*
static void nvram_handler(mame_file *file, int read_or_write)
{
    int i;

    if (read_or_write)
        mame_fwrite(file, main_ram, 0x10000);
    else if (file)
        mame_fread(file, main_ram, 0x10000);
    else
        for (i = 0; i < 0x10000/2; i++)
            main_ram[i] = rand();
}
*/

/**************

Hiscores:

Bloxeed  @ f400-????            [key = ???]
Columns  @ fc00-ffff            [key = '(C) SEGA 1990.JAN BY.TAKOSUKEZOU' @ fc00,ffe0]
Columns2 @ fc00-ffff            [key = '(C) SEGA 1990.SEP.COLUMNS2 JAPAN' @ fc00,fd00,fe00,ffe0]
Borench  @ f400-f5ff            [key = 'EIJI' in last word]
TForceAC @ 8100-817f/8180-81ff  [key = '(c)Tehcno soft90' @ 8070 and 80f0]
TantR    @ fc00-fcff/fd00-fdff  [key = 0xd483 in last word]
PuyoPuyo @ fc00-fdff/fe00-ffff  [key = 0x28e1 in first word]
Ichidant @ fc00-fcff/fd00-fdff  [key = 0x85a9 in last word]
StkClmns @ fc00-fc7f/fc80-fcff  [key = ???]
PuyoPuy2
PotoPoto
ZunkYou

***************/



/******************************************************************************
    Memory Maps
*******************************************************************************

    The System C/C2 68k Memory map is fairly similar to the Genesis in terms
    of RAM, ROM, VDP access locations, although the differences between the
    arcade system and the Genesis means its not same.

******************************************************************************/
/*
static ADDRESS_MAP_START( main_map, ADDRESS_SPACE_PROGRAM, 16 )
	AM_RANGE(0x000000, 0x1fffff) AM_ROM
	AM_RANGE(0x800000, 0x800001) AM_MIRROR(0x13fdfe) AM_READWRITE(prot_r, prot_w)
	AM_RANGE(0x800200, 0x800201) AM_MIRROR(0x13fdfe) AM_WRITE(control_w)
	AM_RANGE(0x840000, 0x84001f) AM_MIRROR(0x13fee0) AM_READWRITE(io_chip_r, io_chip_w)
	AM_RANGE(0x840100, 0x840107) AM_MIRROR(0x13fef8) AM_READWRITE(ym3438_r, ym3438_w)
	AM_RANGE(0x880000, 0x880001) AM_MIRROR(0x13fefe) AM_WRITE(segac2new_upd7759_w)
	AM_RANGE(0x880100, 0x880101) AM_MIRROR(0x13fefe) AM_WRITE(counter_timer_w)
	AM_RANGE(0x8c0000, 0x8c0fff) AM_MIRROR(0x13f000) AM_READWRITE(palette_r, palette_w) AM_BASE(&paletteram16)
	AM_RANGE(0xc00000, 0xc0001f) AM_MIRROR(0x18ff00) AM_READWRITE(genesis_vdp_r, genesis_vdp_w)
	AM_RANGE(0xc00010, 0xc00017) AM_MIRROR(0x18ff00) AM_WRITE(sn76489_w)
	AM_RANGE(0xe00000, 0xe0ffff) AM_MIRROR(0x1f0000) AM_RAM AM_BASE(&generic_nvram16) AM_SIZE(&generic_nvram_size)
ADDRESS_MAP_END
*/
static MEMORY_READ16_START( main_readmem )
    { 0x000000, 0x1fffff, MRA16_ROM },
	{ 0x800000, 0x800001, prot_r },
	{ 0x93fdfe, 0x93fdff, prot_r }, //mirror
	{ 0x840000, 0x84001f, io_chip_r },
	{ 0x97fee0, 0x97feff, io_chip_r }, //mirror
	{ 0x840100, 0x840107, ym3438_r },
	{ 0x97fff8, 0x97ffff, ym3438_r }, //mirror
	{ 0x8c0000, 0x8c0fff, palette_r },
	{ 0x9ff000, 0x9fffff, palette_r }, //mirror
	{ 0xc00000, 0xc0001f, genesis_vdp_r },
	{ 0xd8ff00, 0xd8ff1f, genesis_vdp_r }, //mirror
	{ 0xe00000, 0xe0ffff, MRA16_RAM },
	{ 0xff0000, 0xffffff, MRA16_RAM }, //mirror
MEMORY_END

static MEMORY_WRITE16_START( main_writemem )
    { 0x000000, 0x1fffff, MWA16_ROM },
	{ 0x800000, 0x800001, prot_w },
	{ 0x93fdfe, 0x93fdff, prot_w }, //mirror
	{ 0x800200, 0x800201, control_w },
	{ 0x93fffe, 0x93ffff, control_w }, //mirror
	{ 0x840000, 0x84001f, io_chip_w },
	{ 0x97fee0, 0x97feff, io_chip_w }, //mirror
	{ 0x840100, 0x840107, ym3438_w },
	{ 0x97fff8, 0x97ffff, ym3438_w }, //mirror
	{ 0x880000, 0x880001, segac2new_upd7759_w },
	{ 0x9bfefe, 0x9bfeff, segac2new_upd7759_w }, //mirror
	{ 0x880100, 0x880101, counter_timer_w },
	{ 0x9bfffe, 0x9bffff, counter_timer_w }, //mirror
	{ 0x8c0000, 0x8c0fff, palette_w, &paletteram16 },
	{ 0x9ff000, 0x9fffff, palette_w, &paletteram16 }, //mirror
	{ 0xc00000, 0xc0001f, genesis_vdp_w },
	{ 0xd8ff00, 0xd8ff1f, genesis_vdp_w }, //mirror
	{ 0xc00010, 0xc00017, sn76489_w },
	{ 0xd8ff10, 0xd8ff17, sn76489_w }, //mirror
	{ 0xe00000, 0xe0ffff, MWA16_RAM, (data16_t **)&generic_nvram, &generic_nvram_size },
	{ 0xff0000, 0xffffff, MWA16_RAM, (data16_t **)&generic_nvram, &generic_nvram_size }, //mirror
MEMORY_END


/******************************************************************************
    Input Ports
*******************************************************************************

    The input ports on the C2 games always consist of 1 Coin Port, 2 Player
    Input ports and 2 Dipswitch Ports, 1 of those Dipswitch Ports being used
    for coinage, the other for Game Options.

    Most of the Games List the Dipswitchs and Inputs in the Test Menus, adding
    them is just a tedious task.  I think Columnns & Bloxeed are Exceptions
    and will need their Dipswitches working out by observation.  The Coin Part
    of the DSW's seems fairly common to all games.

******************************************************************************/


INPUT_PORTS_START( twinsqua )
    PORT_START
	PORT_ANALOG( 0xff, 0x00, IPT_DIAL | IPF_PLAYER1, 30, 15, 0, 0)

    PORT_START
	PORT_ANALOG( 0xff, 0x00, IPT_DIAL | IPF_PLAYER2, 30, 15, 0, 0)
	
	PORT_START
	PORT_BIT( 0x3f, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_SPECIAL )	/* From uPD7759 pin 18. (/BUSY output) */
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_SPECIAL )	/* From MB3773P pin 1. (/RESET output) */

	PORT_START
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START
    PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_COIN1 )
    PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_SERVICE_NO_TOGGLE( 0x04, IP_ACTIVE_LOW )
    PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_SERVICE1 )
    PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_START1 )
    PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON1 | IPF_PLAYER1 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON1 | IPF_PLAYER2 )


	PORT_START
	PORT_DIPNAME( 0x0f, 0x0f, DEF_STR( Coin_A ) )
	PORT_DIPSETTING(    0x07, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x09, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x05, "2 Coins/1 Credit 5/3 6/4" )
	PORT_DIPSETTING(    0x04, "2 Coins/1 Credit 4/3" )
	PORT_DIPSETTING(    0x0f, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x01, "1 Coin/1 Credit 2/3" )
	PORT_DIPSETTING(    0x02, "1 Coin/1 Credit 4/5" )
	PORT_DIPSETTING(    0x03, "1 Coin/1 Credit 5/6" )
	PORT_DIPSETTING(    0x06, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(    0x0e, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x0d, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x0c, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(    0x0b, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(    0x0a, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(    0x00, "Free Play (if Coin B too) or 1/1" )
	PORT_DIPNAME( 0xf0, 0xf0, DEF_STR( Coin_B ) )
	PORT_DIPSETTING(    0x70, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x80, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x90, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x50, "2 Coins/1 Credit 5/3 6/4" )
	PORT_DIPSETTING(    0x40, "2 Coins/1 Credit 4/3" )
	PORT_DIPSETTING(    0xf0, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x10, "1 Coin/1 Credit 2/3" )
	PORT_DIPSETTING(    0x20, "1 Coin/1 Credit 4/5" )
	PORT_DIPSETTING(    0x30, "1 Coin/1 Credit 5/6" )
	PORT_DIPSETTING(    0x60, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(    0xe0, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0xd0, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0xc0, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(    0xb0, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(    0xa0, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(    0x00, "Free Play (if Coin A too) or 1/1" )

	PORT_START
	PORT_DIPNAME( 0x01, 0x01, "Credits to Start" )
	PORT_DIPSETTING(    0x01, "1" )
	PORT_DIPSETTING(    0x00, "2" )
    PORT_DIPNAME( 0x02, 0x00, DEF_STR( Demo_Sounds ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
    PORT_DIPNAME( 0x04, 0x04, "Buy In" )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x18, 0x18, DEF_STR( Difficulty ) )
	PORT_DIPSETTING(    0x10, "Easy" )
    PORT_DIPSETTING(    0x18, "Medium" )
	PORT_DIPSETTING(    0x08, "Hard" )
	PORT_DIPSETTING(    0x00, "Hardest" )
    PORT_DIPNAME( 0x20, 0x20, "Seat Type" )
	PORT_DIPSETTING(    0x20, "Normal" )
	PORT_DIPSETTING(    0x00, "Moving" )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END


INPUT_PORTS_START( ooparts )
    PORT_START
    PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 | IPF_PLAYER1 )
    PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNUSED )		/* Button 2 Unused */
    PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED )		/* Button 3 Unused */
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN )
    PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN | IPF_PLAYER1 )
    PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_JOYSTICK_UP | IPF_PLAYER1 )
    PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT | IPF_PLAYER1)
    PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT | IPF_PLAYER1)

    PORT_START
    PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 | IPF_PLAYER2 )
    PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNUSED )		/* Button 2 Unused */
    PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED )		/* Button 3 Unused */
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN )
    PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN | IPF_PLAYER2 )
    PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_JOYSTICK_UP | IPF_PLAYER2 )
    PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT | IPF_PLAYER2 )
    PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT | IPF_PLAYER2 )

	PORT_START
	PORT_BIT( 0x3f, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_SPECIAL )	/* From uPD7759 pin 18. (/BUSY output) */
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_SPECIAL )	/* From MB3773P pin 1. (/RESET output) */

	PORT_START
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START
    PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_COIN1 )
    PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_SERVICE_NO_TOGGLE( 0x04, IP_ACTIVE_LOW )
    PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_SERVICE1 )
    PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_START1 )
    PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_START2 )
    PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
    PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START
	PORT_DIPNAME( 0x0f, 0x0f, DEF_STR( Coin_A ) )
	PORT_DIPSETTING(    0x07, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x09, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x05, "2 Coins/1 Credit 5/3 6/4" )
	PORT_DIPSETTING(    0x04, "2 Coins/1 Credit 4/3" )
	PORT_DIPSETTING(    0x0f, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x01, "1 Coin/1 Credit 2/3" )
	PORT_DIPSETTING(    0x02, "1 Coin/1 Credit 4/5" )
	PORT_DIPSETTING(    0x03, "1 Coin/1 Credit 5/6" )
	PORT_DIPSETTING(    0x06, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(    0x0e, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x0d, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x0c, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(    0x0b, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(    0x0a, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(    0x00, "Free Play (if Coin B too) or 1/1" )
	PORT_DIPNAME( 0xf0, 0xf0, DEF_STR( Coin_B ) )
	PORT_DIPSETTING(    0x70, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x80, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x90, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x50, "2 Coins/1 Credit 5/3 6/4" )
	PORT_DIPSETTING(    0x40, "2 Coins/1 Credit 4/3" )
	PORT_DIPSETTING(    0xf0, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x10, "1 Coin/1 Credit 2/3" )
	PORT_DIPSETTING(    0x20, "1 Coin/1 Credit 4/5" )
	PORT_DIPSETTING(    0x30, "1 Coin/1 Credit 5/6" )
	PORT_DIPSETTING(    0x60, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(    0xe0, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0xd0, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0xc0, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(    0xb0, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(    0xa0, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(    0x00, "Free Play (if Coin A too) or 1/1" )

	PORT_START
    PORT_DIPNAME( 0x01, 0x00, DEF_STR( Demo_Sounds ) )
    PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x06, 0x06, DEF_STR( Difficulty ) )
    PORT_DIPSETTING(    0x04, "Easy" )
    PORT_DIPSETTING(    0x06, "Medium" )
    PORT_DIPSETTING(    0x02, "Hard" )
	PORT_DIPSETTING(    0x00, "Hardest" )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END



/******************************************************************************
    Sound interfaces
******************************************************************************/

static struct UPD7759_interface upd7759_intf =
{
	1,								/* One chip */
	{ 75 },							/* Volume */
	{ REGION_SOUND1 },				/* Memory pointer (gen.h) */
	UPD7759_STANDALONE_MODE			/* Chip mode */
};

static struct YM2612interface ym3438_intf =
{
	1,								/* One chip */
	XL2_CLOCK/7,					/* Clock: 7.67 MHz */
	{ YM3012_VOL(50,MIXER_PAN_CENTER,50,MIXER_PAN_CENTER) },	/* Volume */
	{ 0 },							/* port I/O */
	{ 0 },							/* port I/O */
	{ 0 },							/* port I/O */
	{ 0 },							/* port I/O */
	{ genesis_irq2_interrupt }			/* IRQ handler */
};


static struct SN76496interface sn76489_intf =
{
	1,								/* One chip */
	{ XL2_CLOCK/15 },			/* Clock: 3.58 MHz */
	{ 50 }							/* Volume */
};



/******************************************************************************
    Machine Drivers
*******************************************************************************

    General Overview
        M68000 @ 10MHz (Main Processor)
        YM3438 (Fm Sound)
        SN76489 (PSG, Noise, Part of the VDP)
        UPD7759 (Sample Playback, C-2 Only)

******************************************************************************/
static MACHINE_DRIVER_START( segacnew )

	/* basic machine hardware */
	MDRV_CPU_ADD_TAG("main", M68000, XL2_CLOCK/6)
	MDRV_CPU_MEMORY(main_readmem, main_writemem)
	MDRV_CPU_VBLANK_INT(genesis_vblank_interrupt,1)

	MDRV_FRAMES_PER_SECOND(60)
	MDRV_VBLANK_DURATION((int)(((262. - 224.) / 262.) * 1000000. / 60.))

	MDRV_MACHINE_INIT(segac2new)
/*	MDRV_NVRAM_HANDLER(generic_randfill) not supported in MAME78 */

	/* video hardware */
	MDRV_VIDEO_ATTRIBUTES(VIDEO_TYPE_RASTER | VIDEO_HAS_SHADOWS | VIDEO_HAS_HIGHLIGHTS)
	MDRV_SCREEN_SIZE(320,224)
	MDRV_VISIBLE_AREA(0, 319, 0, 223)
	MDRV_PALETTE_LENGTH(2048)

	MDRV_VIDEO_START(segac2new)
	MDRV_VIDEO_UPDATE(segac2new)

	/* sound hardware */
	MDRV_SOUND_ADD(YM2612, ym3438_intf)
	MDRV_SOUND_ADD(SN76496, sn76489_intf)
	MDRV_SOUND_ADD(UPD7759, upd7759_intf)
MACHINE_DRIVER_END




/******************************************************************************
    Rom Definitions
*******************************************************************************

    All the known System C/C2 Dumps are listed here with the exception of
    the version of Puzzle & Action (I believe its actually Ichidant-R) which
    was credited to SpainDumps in the included text file.  This appears to be
    a bad dump (half sized roms) however the roms do not match up exactly with
    the good dump of the game.  English language sets are assumed to be the
    parent where they exist.  Hopefully some more alternate version dumps will
    turn up sometime soon for example English Language version of Tant-R or
    Japanese Language versions of Borench (if of course these games were
    released in other locations.

    Games are in Order of Date (Year) with System-C titles coming first.

******************************************************************************/


/* ----- System C2 Games ----- */

ROM_START( headonch ) /* Head On Channel (Prototype) (c)1994 Sega */
	ROM_REGION( 0x200000, REGION_CPU1, 0 )
	ROM_LOAD16_BYTE( "epr-16812.ic32", 0x000000, 0x080000, CRC(091cf538) )
	ROM_LOAD16_BYTE( "epr-16811.ic31", 0x000001, 0x080000, CRC(91f3b5f1) )
	ROM_LOAD16_BYTE( "epr-16814.ic34", 0x100000, 0x080000, CRC(d8dc6323) )
	ROM_LOAD16_BYTE( "epr-16813.ic33", 0x100001, 0x080000, CRC(3268e38b) )

	ROM_REGION( 0x040000, REGION_SOUND1, 0 )
	ROM_LOAD( "epr-16810.ic4", 0x000000, 0x040000, CRC(90af7301) )
ROM_END

ROM_START( ooparts ) /* Oo Parts (c)1992 Sega / Success */
	ROM_REGION( 0x200000, REGION_CPU1, 0 )
	ROM_LOAD16_BYTE( "epr-15614.ic32", 0x000000, 0x080000, CRC(8dcf2940)  )
	ROM_LOAD16_BYTE( "epr-15613.ic31", 0x000001, 0x080000, CRC(35381899)  )
	ROM_LOAD16_BYTE( "mpr-15616.ic34", 0x100000, 0x080000, CRC(7192ac29)  )
	ROM_LOAD16_BYTE( "mpr-15615.ic33", 0x100001, 0x080000, CRC(42755dc2)  )

	ROM_REGION( 0x040000, REGION_SOUND1, 0 )
	ROM_LOAD( "epr-15617.ic4", 0x000000, 0x040000, CRC(e09961f6)  )
ROM_END

ROM_START( twinsqua ) /* Twin Squash  (c)1991 Sega */
	ROM_REGION( 0x200000, REGION_CPU1, 0 )
	ROM_LOAD16_BYTE( "ep14657.32", 0x000000, 0x040000, CRC(becbb1a1) SHA1(787b1a4bf420186d05b5448582f6492e40d394fa) )
	ROM_LOAD16_BYTE( "ep14656.31", 0x000001, 0x040000, CRC(411906e7) SHA1(68a4e66b9e18499d77cdb584470f35f67edec6fd) )

	ROM_REGION( 0x020000, REGION_SOUND1, 0 )
	ROM_LOAD( "ep14588.4", 0x000000, 0x020000, CRC(5a9b6881) SHA1(d86ec7f569fae5a1ce93a1cf40998cbb13726e0c) )
ROM_END


/******************************************************************************
    Machine Init Functions
*******************************************************************************

    All of the Sega C/C2 games apart from Bloxeed used a protection chip.
    The games contain various checks which make sure this protection chip is
    present and returning the expected values.  The chip uses a table of
    256x4-bit values to produce its results.  It appears that different
    tables are used for Japanese vs. English variants of some games
    (Puzzle & Action 2) but not others (Columns).

******************************************************************************/

static void common_init(const UINT32 *table)
{
	prot_table = table;
	bloxeed_sound = 0;
	
	state_save_register_UINT8 ("genesis", 0, "Int 2 Status", &irq2_int, 1);
	state_save_register_UINT8 ("genesis", 0, "Int 4 Status", &scanline_int, 1);
	state_save_register_UINT8 ("genesis", 0, "Int 6 Status", &vblank_int, 1);

	state_save_register_UINT8("C2_IO", 0, "I/O Writes", misc_io_data, 0x10);
	state_save_register_UINT8("C2 Protection", 0, "Write Buffer", &prot_write_buf, 1);
	state_save_register_UINT8("C2 Protection", 0, "Read Buffer", &prot_read_buf, 1);
}


static DRIVER_INIT( twinsqua )
{
	static const UINT32 table[256/8] =
	{
		0xbb33aa22, 0xffffeeee, 0xa820bb33, 0xfd75ee66,
		0xbb33bb33, 0xbbbbbbbb, 0xa820aa22, 0xb931bb33,
		0x33bb22aa, 0xffffeeee, 0x22aa31b9, 0x77ff64ec,
		0x33bb33bb, 0xbbbbbbbb, 0x22aa20a8, 0x33bb31b9,
		0xbb33aa22, 0xffffeeee, 0xec64ff77, 0xb931aa22,
		0xbb33bb33, 0xbbbbbbbb, 0xec64ee66, 0xfd75ff77,
		0x33bb22aa, 0xffffeeee, 0x66ee75fd, 0x33bb20a8,
		0x33bb33bb, 0xbbbbbbbb, 0x66ee64ec, 0x77ff75fd
	};
	common_init(table);
}


static DRIVER_INIT( ooparts )
{
	static const UINT32 table[256/8] =
	{
		0x91ddd19d, 0x91ddd19d, 0xd4dc949c, 0xf6feb6be,
		0x91bbd1fb, 0x91bbd1fb, 0xd4fe94be, 0xf6feb6be,
		0x80cce2ae, 0x88cceaae, 0xc5cda7af, 0xefef8d8d,
		0x91bbf3d9, 0x99bbfbd9, 0xd4feb69c, 0xfefe9c9c,
		0x5d55959d, 0x5d55959d, 0x5c54949c, 0x7e76b6be,
		0x5d7795bf, 0x5d7795bf, 0x5c7694be, 0x7e76b6be,
		0x5d55b7bf, 0x4444aeae, 0x5c54b6be, 0x67678d8d,
		0x5d77b79d, 0x5577bf9d, 0x5c76b69c, 0x76769c9c
	};
	common_init(table);
}



/******************************************************************************
    Game Drivers
*******************************************************************************

    These cover all the above games.

    Dates are all verified correct from Ingame display, some of the Titles
    such as Ichidant-R, Tant-R might be slightly incorrect as I've seen the
    games refered to by other names such as Ichident-R, Tanto-R, Tanto Arle
    etc.

    bloxeedc is set as as clone of bloxeed as it is the same game but running
    on a different piece of hardware.  The parent 'bloxeed' is a system18 game
    and does not currently work due to it being encrypted.

******************************************************************************/

GAME ( 1994, headonch, 0,  segacnew, ooparts,  ooparts,  ROT0, "Sega", "Head On Channel (Japan, prototype)" )
GAME ( 1992, ooparts,  0,  segacnew, ooparts,  ooparts,  ROT270, "Sega / Success",   "Oo Parts (Japan, Prototype)" )
GAME ( 1991, twinsqua, 0,  segacnew, twinsqua, twinsqua, ROT0, "Sega",   "Twin Squash" )
