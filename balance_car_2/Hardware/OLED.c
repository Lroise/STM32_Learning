#include "oled.h"
#include "iic.h"
#include "oledfont.h"  
#include "bmp.h"
/****************************************************************************
*函数名：Write_IIC_Command
*输  入：IIC_Command：传入命令
*输  出：无
*功  能：IIC写入命令
****************************************************************************/
void Write_IIC_Command(unsigned char IIC_Command)
{
    iicbus_start();
    iicbus_send_byte(0x78);            //Slave address,SA0=0
    iicbus_wait_ack();  
    iicbus_send_byte(0x00);             //write command
    iicbus_wait_ack();  
    iicbus_send_byte(IIC_Command); 
    iicbus_wait_ack();  
    iicbus_stop();
}
/****************************************************************************
*函数名：Write_IIC_Data
*输  入：IIC_Data：传入数据
*输  出：无
*功  能：IIC写入数据
****************************************************************************/
void Write_IIC_Data(unsigned char IIC_Data)
{
    iicbus_start();
    iicbus_send_byte(0x78);         //D/C#=0; R/W#=0
    iicbus_wait_ack();      
    iicbus_send_byte(0x40);         //write data
    iicbus_wait_ack();  
    iicbus_send_byte(IIC_Data);
    iicbus_wait_ack();  
    iicbus_stop();
}

/****************************************************************************
*函数名：OLED_WR_Byte
*输  入：dat:数据、 cmd:1为数据、0为命令
*输  出：无
*功  能：OLED写入数据、命令
****************************************************************************/
void OLED_WR_Byte(unsigned dat,unsigned cmd)
{
    if(cmd) {
        Write_IIC_Data(dat);
    } else {
        Write_IIC_Command(dat); 
    }

}

/****************************************************************************
*函数名：fill_picture
*输  入：fill_Data:填充图片数据
*输  出：无
*功  能：绘画图片
****************************************************************************/
void fill_picture(unsigned char fill_Data)
{
    unsigned char m,n;
    for(m=0;m<8;m++)
    {
        OLED_WR_Byte(0xb0+m,0);		//page0-page1
        OLED_WR_Byte(0x00,0);		//low column start address
        OLED_WR_Byte(0x10,0);		//high column start address
        for(n=0;n<128;n++)
            {
                OLED_WR_Byte(fill_Data,1);
            }
    }
}


/****************************************************************************
*函数名：Delay_50ms、Delay_1ms
*输  入：延时时间
*输  出：无
*功  能：延时
****************************************************************************/
void Delay_50ms(unsigned int Del_50ms)
{
    unsigned int m;
    for(;Del_50ms>0;Del_50ms--)
        for(m=6245;m>0;m--);
}

void Delay_1ms(unsigned int Del_1ms)
{
    unsigned char j;
    while(Del_1ms--)
    {   
        for(j=0;j<123;j++);
    }
}

/****************************************************************************
*函数名：OLED_Set_Pos
*输  入：x:X轴坐标、y:y轴坐标
*输  出：无
*功  能：坐标位置设置
****************************************************************************/
void OLED_Set_Pos(unsigned char x, unsigned char y) 
{
    OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&0x0f),OLED_CMD); 
}

/****************************************************************************
*函数名：OLED_Display_On
*输  入：无
*输  出：无
*功  能：开启OLED显示 
****************************************************************************/
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}

/****************************************************************************
*函数名：OLED_Display_Off
*输  入：无
*输  出：无
*功  能：关闭OLED显示
****************************************************************************/    
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}

/****************************************************************************
*函数名：OLED_Clear
*输  入：无
*输  出：无
*功  能：清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!   
****************************************************************************/
void OLED_Clear(void)  
{  
    uint8_t i,n; 
    for(i=0;i<8;i++)  
    {  
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
        OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
        OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
        for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
    } //更新显示
}
/****************************************************************************
*函数名：OLED_On
*输  入：无
*输  出：无
*功  能：点亮屏幕，白屏  
****************************************************************************/
void OLED_On(void)  
{  
    uint8_t i,n;  
    for(i=0;i<8;i++)  
    {  
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
        OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
        OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
        for(n=0;n<128;n++)OLED_WR_Byte(1,OLED_DATA); 
    } 
}
 /****************************************************************************
*函数名：OLED_ShowChar
*输  入：x:0~127 、y:0~63 、mode:0,反白显示;1,正常显示、size:选择字体 16/12 
*输  出：无
*功  能：点亮屏幕，白屏  
****************************************************************************/
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size)
{      	
    unsigned char c=0,i=0;	
    c=chr-' ';//得到偏移后的值 
    if(x>Max_Column-1){x=0;y=y+2;}
    if(Char_Size ==16)
        {
        OLED_Set_Pos(x,y);  
        for(i=0;i<8;i++)
        OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
        OLED_Set_Pos(x,y+1);
        for(i=0;i<8;i++)
        OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
        }
        else {	
            OLED_Set_Pos(x,y);
            for(i=0;i<6;i++)
            OLED_WR_Byte(F6x8[c][i],OLED_DATA);
            
        }
}

/****************************************************************************
*函数名：oled_pow
*输  入：次方函数M的N次方 
*输  出：无
*功  能：m^n函数
****************************************************************************/
uint32_t oled_pow(uint8_t m,uint8_t n)
{
    uint32_t result=1;	 
    while(n--)result*=m;    
    return result;
}


/****************************************************************************
*函数名：OLED_ShowNum
*输  入：x,y :起点坐标、 len :数字的位数、size:字体大小
         mode:模式	0,填充模式;1,叠加模式、num:数值(0~4294967295);
*输  出：无
*功  能：显示len个数字
****************************************************************************/

void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size2)
{           
    uint8_t t,temp;
    uint8_t enshow=0;    
    for(t=0;t<len;t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
                continue;
            }else enshow=1; 
             
        }
        OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2); 
    }
}
/****************************************************************************
*函数名：OLED_ShowString
*输  入：x,y :起点坐标、 chr :字符串空间位置、Char_Size:字体大小
*输  出：无
*功  能：显示一个字符号串
****************************************************************************/

void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size)
{
    unsigned char j=0;
    while (chr[j]!='\0')
    {   OLED_ShowChar(x,y,chr[j],Char_Size);
            x+=8;
        if(x>120){x=0;y+=2;}
            j++;
    }
}
/****************************************************************************
*函数名：OLED_ShowCHinese
*输  入：x,y :起点坐标、 no:字体位置
*输  出：无
*功  能：显示汉字
****************************************************************************/
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no)
{   
    uint8_t t,adder=0;
    OLED_Set_Pos(x,y);
    for(t=0;t<16;t++) {
        OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
        adder+=1;
     }
        OLED_Set_Pos(x,y+1);
    for(t=0;t<16;t++)
    {
        OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
        adder+=1;
    }
}

/****************************************************************************
*函数名：OLED_DrawBMP
*输  入：128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7
*输  出：无
*功  能：显示显示BMP图片
****************************************************************************/

void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{ 
    unsigned int j=0;
    unsigned char x,y;

    if(y1%8==0) y=y1/8;      
    else y=y1/8+1;
    for(y=y0;y<y1;y++)
    {
        OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
        {      
            OLED_WR_Byte(BMP[j++],OLED_DATA);
        }
    }
} 
/****************************************************************************
*函数名：OLED_Init
*输  入：无
*输  出：无
*功  能：初始化SSD1306
****************************************************************************/   
void OLED_Init(void)
{
    iicbus_init();
    delay_ms(200);
    OLED_WR_Byte(0xAE, OLED_CMD); //--display off
    OLED_WR_Byte(0x00, OLED_CMD); //---set low column address
    OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
    OLED_WR_Byte(0x40, OLED_CMD); //--set start line address
    OLED_WR_Byte(0xB0, OLED_CMD); //--set page address
    OLED_WR_Byte(0x81, OLED_CMD); // contract control
    OLED_WR_Byte(0xFF, OLED_CMD); //--128
    OLED_WR_Byte(0xA1, OLED_CMD); //set segment remap
    OLED_WR_Byte(0xA6, OLED_CMD); //--normal / reverse
    OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3F, OLED_CMD); //--1/32 duty
    OLED_WR_Byte(0xC8, OLED_CMD); //Com scan direction
    OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset
    OLED_WR_Byte(0x00, OLED_CMD); //

    OLED_WR_Byte(0xD5, OLED_CMD); //set osc division
    OLED_WR_Byte(0x80, OLED_CMD); //

    OLED_WR_Byte(0xD8, OLED_CMD); //set area color mode off
    OLED_WR_Byte(0x05, OLED_CMD); //

    OLED_WR_Byte(0xD9, OLED_CMD); //Set Pre-Charge Period
    OLED_WR_Byte(0xF1, OLED_CMD); //

    OLED_WR_Byte(0xDA, OLED_CMD); //set com pin configuartion
    OLED_WR_Byte(0x12, OLED_CMD); //

    OLED_WR_Byte(0xDB, OLED_CMD); //set Vcomh
    OLED_WR_Byte(0x30, OLED_CMD); //

    OLED_WR_Byte(0x8D, OLED_CMD); //set charge pump enable
    OLED_WR_Byte(0x14, OLED_CMD); //

    OLED_WR_Byte(0xAF, OLED_CMD); //--turn on oled panel
    OLED_Clear();
}  
