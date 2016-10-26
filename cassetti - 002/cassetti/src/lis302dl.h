
#define			LIS302DL_ADDRESS				 0x1C


#define			 START_REG  					  0x00//00
#define	         LIS302DL_REG_WHO_AM_I            0x0f//15
#define	         LIS302DL_REG_CTRL1               0x20//32
#define	         LIS302DL_REG_CTRL2               0x21//33
#define	         LIS302DL_REG_CTRL3               0x22//34
#define	         LIS302DL_REG_HP_FILTER_RESET     0x23//35
#define	         LIS302DL_REG_STATUS              0x27//39
#define	         LIS302DL_REG_OUT_X               0x29//41
#define	         LIS302DL_REG_OUT_Y               0x2b//43
#define	         LIS302DL_REG_OUT_Z               0x2d//45
#define	         LIS302DL_REG_FF_WU_CFG_1         0x30//48
#define	         LIS302DL_REG_FF_WU_SRC_1         0x31//49
#define	         LIS302DL_REG_FF_WU_THS_1         0x32//50
#define	         LIS302DL_REG_FF_WU_DURATION_1    0x33//51
#define	         LIS302DL_REG_FF_WU_CFG_2         0x34//52
#define	         LIS302DL_REG_FF_WU_SRC_2         0x35//53
#define	         LIS302DL_REG_FF_WU_THS_2         0x36//54
#define	         LIS302DL_REG_FF_WU_DURATION_2    0x37//55
#define	         LIS302DL_REG_CLICK_CFG           0x38//56
#define	         LIS302DL_REG_CLICK_SRC           0x39//57
#define	         LIS302DL_REG_CLICK_THSY_X        0x3b//59
#define	         LIS302DL_REG_CLICK_THSZ          0x3c//60
#define	         LIS302DL_REG_CLICK_TIME_LIMIT    0x3d//61
#define	         LIS302DL_REG_CLICK_LATENCY       0x3e//62
#define	         LIS302DL_REG_CLICK_WINDOW        0x3f//63


#define	         LIS302DL_CTRL1_Xen               0x01
#define	         LIS302DL_CTRL1_Yen               0x02
#define	         LIS302DL_CTRL1_Zen               0x04
#define	         LIS302DL_CTRL1_STM               0x08
#define	         LIS302DL_CTRL1_STP               0x10
#define	         LIS302DL_CTRL1_FS                0x20
#define	         LIS302DL_CTRL1_PD                0x40
#define	         LIS302DL_CTRL1_DR                0x80

#define	         LIS302DL_CTRL2_HPC1              0x01
#define	         LIS302DL_CTRL2_HPC2              0x02
#define	         LIS302DL_CTRL2_HPFF1             0x04
#define	         LIS302DL_CTRL2_HPFF2             0x08
#define	         LIS302DL_CTRL2_FDS               0x10
#define	         LIS302DL_CTRL2_BOOT              0x40
#define	         LIS302DL_CTRL2_SIM               0x80


#define	         LIS302DL_CTRL3_PP_OD             0x40
#define	         LIS302DL_CTRL3_IHL               0x80


#define	         LIS302DL_STATUS_XDA              0x01
#define	         LIS302DL_STATUS_YDA              0x02
#define	         LIS302DL_STATUS_ZDA              0x04
#define	         LIS302DL_STATUS_XYZDA            0x08
#define	         LIS302DL_STATUS_XOR              0x10
#define	         LIS302DL_STATUS_YOR              0x20
#define	         LIS302DL_STATUS_ZOR              0x40
#define	         LIS302DL_STATUS_XYZOR            0x80

 /* Wakeup/freefall interrupt defs */
#define	         LIS302DL_FFWUCFG_XLIE            0x01
#define	         LIS302DL_FFWUCFG_XHIE            0x02
#define	         LIS302DL_FFWUCFG_YLIE            0x04
#define	         LIS302DL_FFWUCFG_YHIE            0x08
#define	         LIS302DL_FFWUCFG_ZLIE            0x10
#define	         LIS302DL_FFWUCFG_ZHIE            0x20
#define	         LIS302DL_FFWUCFG_LIR             0x40
#define	         LIS302DL_FFWUCFG_AOI             0x80



#define	         LIS302DL_FFWUTHS_DCRM            0x80



#define	         LIS302DL_FFWUSRC_XL              0x01
#define	         LIS302DL_FFWUSRC_XH              0x02
#define	         LIS302DL_FFWUSRC_YL              0x04
#define	         LIS302DL_FFWUSRC_YH              0x08
#define	         LIS302DL_FFWUSRC_ZL              0x10
#define         LIS302DL_FFWUSRC_ZH               0x20
#define	         LIS302DL_FFWUSRC_IA              0x40



#define	         LIS302DL_CLICKSRC_SINGLE_X       0x01
#define	         LIS302DL_CLICKSRC_DOUBLE_X       0x02
#define	         LIS302DL_CLICKSRC_SINGLE_Y       0x04
#define	         LIS302DL_CLICKSRC_DOUBLE_Y       0x08
#define	         LIS302DL_CLICKSRC_SINGLE_Z       0x10
#define	         LIS302DL_CLICKSRC_DOUBLE_Z       0x20
#define         LIS302DL_CLICKSRC_IA              0x40


 #define LIS302DL_WHO_AM_I_MAGIC         0x3b

 #define LIS302DL_F_WUP_FF_1             0x0001  /* wake up from free fall */
 #define LIS302DL_F_WUP_FF_2             0x0002
 #define LIS302DL_F_WUP_FF               0x0003
 #define LIS302DL_F_WUP_CLICK			 0x0004
 #define LIS302DL_F_POWER                0x0010
 #define LIS302DL_F_FS                   0x0020  /* ADC full scale */
 #define LIS302DL_F_INPUT_OPEN			 0x0040  /* Set if input device is opened */
 #define LIS302DL_F_IRQ_WAKE			 0x0080  /* IRQ is setup in wake mode */
 #define LIS302DL_F_DR                   0x0100  /* Data rate, 400Hz/100Hz */

void lis302dl_write_configuration(void);

void lis302dl_write_byte(uint8_t reg_address, uint8_t byte_value);

void lis302dl_write_byte(uint8_t reg_address, uint8_t byte_value);

uint8_t lis302dl_read_byte(uint8_t reg_address);