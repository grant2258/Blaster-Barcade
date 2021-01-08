/* Todo, reorganise, cleanup etc.*/


/* In src/drivers/segac2.c */
extern WRITE16_HANDLER( sn76489_w );


/* In src/vidhrdw/genesis.c */
extern UINT8		genesis_vdp_regs[];
extern UINT16		genesis_bg_pal_lookup[];
extern UINT16		genesis_sp_pal_lookup[];
extern UINT16		scanbase;

VIDEO_START( genesis );
VIDEO_START( segac2new );

VIDEO_UPDATE( genesis );
VIDEO_UPDATE( segac2new );

void genesis_enable_display(int enable);

READ16_HANDLER ( genesis_vdp_r );
WRITE16_HANDLER( genesis_vdp_w );


