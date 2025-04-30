#include <stdio.h>
#include <stdlib.h>

#include "al8051ex.h"
#include "isp_define.h"
#include "isp_reg.h"
#include "math.h"

#include "key_vari.h"
#include "al8051_func.h"
#include "flash.h"

#include "isp_func.h"
#include "sensor.h"
#include "osd.h"

#include "ae_task.h"

#include "key_func_ch.h"

#include "update.h"
#include "isr_task.h"
#include "key_task.h"
#include "gui_func.h"

#include "tp2801.h"
#include "utc_update.h"


extern void gui_task(void);
extern void awb_task(void);
extern void wdr_task(void);
extern void update_dnn(void);
extern void hist_data(void);

void osd_print(void);

//========================================================================================================================
//
//========================================================================================================================
void main(void)
{
    rWDT       = 0x00;

#if MCLK_48MHz
    rMCU         = 0x0C;  //  48MHz
    rTVI_CTRL0 = 0x80;
#else
    rMCU         = 0x08;  //  72MHz
    rTVI_CTRL0 = 0x00;
#endif

    gpio_init();
    timer_init();
    uart0_init();
    i2c_init();
    isr_init();

    nvram_Load();
    font_Load();

    isp_init1();
     tp_init ();
    isp_init2();

    delay_ms(10);
    sensor_init_cmos();

	//Neo modify 2025/04/10
	XBYTE[0xF8C0] = 0x00;		//Disable Audio function
	XBYTE[0xF811] = 0x80;		//For compatible wukong2(T35)

    ae_init();
    osd_init();
    init_dnn();
#if CCTV_SOLUTION
    disp_task();
#endif

    isp_dac_off(1);

    // Output script of version. (HW, SW, ISP)
#if CCTV_SOLUTION
//    disp_version();
//    isp_set_timer(0, 10000);
#endif
    isp_set_timer(1, 1500);   // key timer start


#if USE_GUI
    tpLog(" Mitac T30 TP3822A1 + GC2083");
    debug_str("\r\n "Build_Info);
    tpLog(" SW VER : %c%c%c%c",(char)Mitac_SW_VER[0],(char)Mitac_SW_VER[1],(char)Mitac_SW_VER[2],(char)Mitac_SW_VER[3] );
    tpLog(" IQ VER : %c%c",(char)ISP.Version[1][6], (char)ISP.Version[1][7]);
#endif

	if (flash_read_rdsr()==CMD_LOCK)
		tpLog(" Flash Protect Enable : %x",(uint16_t)flash_read_rdsr());
	else
		tpLog(" Flash Protect Disable : %x",(uint16_t)flash_read_rdsr());
			
    //**********************************************************************
    //
    //**********************************************************************
    while(1)
    {
#if USE_UTC
_top:
        //----------------------------------------------------------------------------------------------------------
        //  UTC Update
        if(g_TP2801_Rx_flag && utc_update_flag)
        {
            utc_update();
            goto _top;
        }
#endif

        //----------------------------------------------------------------------------------------------------------
        //  GUI Protocol
        gui_task();


        //----------------------------------------------------------------------------------------------------------
        //  timer process
        timer_scheduler();


        //----------------------------------------------------------------------------------------------------------
        //  V-Sync Update
        if((g_vSync1 == 1) && (g_Gui_Rcv_Flag == 0))
        {
            g_vSync1  = 0;
            g_vSync2 ^= 1;

            if(g_vSync2)
                hist_data();
            else
            {
                isp_histAnalysis();
                isp_ccm_control();
            }

            // AE
            if(g_Gui_Ae_Task)
                ae_task();

            // AWB
            if(g_Gui_Awb_Task)
                awb_task();

            Adaptive_Image_Update();

            if( (g_Awb_CurMode == SET_AWB_NORMAL) && g_vSync2 )
                update_dnn();

            // ICR_Filter
//            icr_ctrl();
//            icr_task();

            if( g_TP2801_Rx_flag || (ISP.Video_Type == MODE_CVBS) )
            {
                g_TP2801_Rx_flag = 0;
                TP2801_Rx();
            }

            osd_print();
        }


        //----------------------------------------------------------------------------------------------------------
        // Key Process
        if(g_key_flag)
        {
            g_key_flag = 0;

#if CCTV_SOLUTION
            key_check();

            if(g_cur_key_value != NONE_KEY)       { isp_kill_timer(0); isp_set_timer(0, 20000); }

            key_win(g_cur_key_value);
#endif
        }
    }
}

void osd_print(void)
{
#if OSD_DEBUG
    uint08_t xdata dbg_ch=1, dbg_ln=0;

    if(g_menu_state == MN_PAGE_NONE)
    {
        switch (dbg_ch)
        {
        #if 1
            case 1:
                tpPrint( dbg_ln++, "ae_sum : %3d %3d", (int)g_ae_sum, (int)g_Cur_Ae_Target);
                tpPrint( dbg_ln++, "Shutter: %4d",   (int)Update.ae_cur_expline);
                tpPrint( dbg_ln++, "A. gain: %9.4f", (float)Update.ae_cur_gain);
//                tpPrint( dbg_ln++, "D. gain: %9.4f", (float)Update.ae_cur_dgain);
                tpPrint( dbg_ln++, "ISP. dgain: %9.4f", (float)Update.ae_cur_isp_dgain);
 //            tpPrint( dbg_ln++, "ISO: %9.4f,  %9.4f",   (float)ISO, (float)g_Max_ISO);
 //            tpPrint( dbg_ln++, "nISO: %3d",   (int)wISO);
 //            tpPrint( dbg_ln++, "AWB FSum : %4d %4d %4d",   (int)rAWB_SUMR, (int)rAWB_SUMG, (int)rAWB_SUMB);
 //            tpPrint( dbg_ln++, "AWB WSum : %4d %4d %4d",   (int)g_Rsum, (int)g_Gsum, (int)g_Bsum);
 //            tpPrint( dbg_ln++, "AWB Gain : %4d %4d %4d",   (int)rAWB_GAIN_R, (int)rAWB_GAIN_G, (int)rAWB_GAIN_B);
//                tpPrint( dbg_ln++, "AWB Cnt : %8.4f",   (float)g_WhiteCount);
                tpPrint( dbg_ln++, "%s:%3d, %s:%3d, %s:%3d", str_LightSource[CTS[0]], (int)(CTW[0] * 100.f + 0.5f),
                            str_LightSource[CTS[1]], (int)(CTW[1] * 100.f + 0.5f),
                            str_LightSource[CTS[2]], (int)(CTW[2] * 100.f + 0.5f));
		tpPrint( dbg_ln++, "SW VER : %s %s", __DATE__,__TIME__);							//Neo Add 2025/04/09
		tpPrint( dbg_ln++, "IQ VER : %c%c", (char)ISP.Version[1][6],(char)ISP.Version[1][7]);		//Neo Add 2025/04/09 (int)ISP.Version[1][6]
            break;
        #elif 0
            case 2:
                tpPrint( dbg_ln, "ae_sum : %3d %3d", (int)g_ae_sum, (int)g_Cur_Ae_Target);
                tpPrint( dbg_ln++, "Shutter: %4d",   (int)Update.ae_cur_expline);
                tpPrint( dbg_ln++, "A. gain: %9.4f", (float)Update.ae_cur_gain);
                tpPrint( dbg_ln++, "D. gain: %9.4f", (float)Update.ae_cur_dgain);
                tpPrint( dbg_ln++, "ISP. dgain: %9.4f", (float)Update.ae_cur_isp_dgain);
                tpPrint( dbg_ln++, "ISO: %9.4f,  %9.4f",   (float)ISO, (float)g_Max_ISO);
                tpPrint( dbg_ln++, "   nISO: %3d",   (int)wISO);
            break;
        #elif 0
            case 3:
                tpPrint( dbg_ln++, "   nISO: %3d",   (int)wISO);
                tpPrint( dbg_ln++, "AWB FSum : %4d %4d %4d",   (int)rAWB_SUMR, (int)rAWB_SUMG, (int)rAWB_SUMB);
                tpPrint( dbg_ln++, "AWB WSum : %4d %4d %4d",   (int)g_Rsum, (int)g_Gsum, (int)g_Bsum);
                tpPrint( dbg_ln++, "AWB Gain : %4d %4d %4d",   (int)rAWB_GAIN_R, (int)rAWB_GAIN_G, (int)rAWB_GAIN_B);
                tpPrint( dbg_ln++, "AWB Cnt : %8.4f",   (float)g_WhiteCount);
                tpPrint( dbg_ln++, "%s:%3d, %s:%3d, %s:%3d", str_LightSource[CTS[0]], (int)(CTW[0] * 100.f + 0.5f),
                            str_LightSource[CTS[1]], (int)(CTW[1] * 100.f + 0.5f),
                            str_LightSource[CTS[2]], (int)(CTW[2] * 100.f + 0.5f));
            break;
        #elif 0
            case 4:
                tpPrint( dbg_ln++, "ISO: Cur:%4.4f, Max:%4.4f",   (float)ISO, (float)g_Max_ISO);
#if USE_PWM_LIGHT
                tpPrint( dbg_ln++, "nISO: %3d, lsv:%3.4f",   (int)wISO, (float)(g_lsv/1122.0f/64.0f));
#else
                tpPrint( dbg_ln++, "nISO: %3d",   (int)wISO);
#endif
                tpPrint( dbg_ln++, "Histo: %4.4f,  %4.4f",   (float)hIn_v, (float)hout_v);
                tpPrint( dbg_ln++, "GB: %4.4f,  %4.4f",   (float)GB_Max, (float)GB_Min);
                tpPrint( dbg_ln++, "RB: %4.4f,  %4.4f",   (float)RB_Max, (float)RB_Min);
                tpPrint( dbg_ln++, "CD: %4.4f,  %4.4f",   (float)GB_Diff, (float)RB_Diff);
                tpPrint( dbg_ln++, "DNN: %s,%s,dbg:%d",   g_Gui_DN_Mode==0 ? "AUTO" : g_Gui_DN_Mode==1 ? "COLOR" :
                                                                        g_Gui_DN_Mode==2 ? "B/W" : "EXT", g_dnn_state ? "B/W" : "Color",
                                                                        (int)g_dnn_dbg );
                tpPrint( dbg_ln++, "LED: %s, %s",   g_Gui_LED_Mode==0 ? "AUTO" : g_Gui_LED_Mode==1 ? "MANUAL" :
                                                        g_Gui_LED_Mode==2 ? "DIMMING" : "CLOSE",
                                                        g_Gui_DN_IR_Light==0 ? "IR" : g_Gui_DN_IR_Light==1 ? "WHITE" : "NONE");
#if USE_PWM_LIGHT
                tpPrint( dbg_ln++, "PWM: min:%d,max:%d", (int)g_pwm_duty_min, (int)g_pwm_duty_max  );
                tpPrint( dbg_ln++, "PWM: cur:%d, OP:%.4f", (int)g_pwm_duty_val, (float)(g_lsv_gap/64.0f/1122.0f)  );
#endif
            break;
        #endif
        }
    }
#endif
}
