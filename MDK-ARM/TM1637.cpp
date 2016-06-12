//  Author:Frankie.Chu
//  Date:9 April,2012
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
//  Modified record:
//
/*******************************************************************************/
#include "TM1637.h"
#include "stm32f1xx_hal.h"
static int8_t TubeTab[] = {0x3f,0x06,0x5b,0x4f,
                           0x66,0x6d,0x7d,0x07,
                           0x7f,0x6f,0x77,0x7c,
                           0x39,0x5e,0x79,0x71, 64};//0~9,A,b,C,d,E,F,-                        
TM1637::TM1637(uint8_t Clk, uint8_t Data)
{
  Clkpin = Clk;
  Datapin = Data;
}

void TM1637::init(void)
{
  clearDisplay();
}
void pinDI_Output() {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = DI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(DI_GPIO_Port, &GPIO_InitStruct);
	
}
void pinDI_Input() {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = DI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(DI_GPIO_Port, &GPIO_InitStruct);
	
}
void TM1637::writeByte(int8_t wr_data)
{
  uint8_t i,count1;   
  for(i=0;i<8;i++)        //sent 8bit data
  {
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);	
    if(wr_data & 0x01) HAL_GPIO_WritePin(DI_GPIO_Port, DI_Pin, GPIO_PIN_SET);	//LSB first
    else HAL_GPIO_WritePin(DI_GPIO_Port, DI_Pin, GPIO_PIN_RESET);
    wr_data >>= 1;      
    HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
      
  }  
  HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);	 //wait for the ACK
  HAL_GPIO_WritePin(DI_GPIO_Port, DI_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);     
  pinDI_Input();
  while(HAL_GPIO_ReadPin(DI_GPIO_Port, DI_Pin))    
  { 
    count1 +=1;
    if(count1 == 200)//
    {
     pinDI_Output();
     HAL_GPIO_WritePin(DI_GPIO_Port, DI_Pin, GPIO_PIN_RESET);
     count1 =0;
    }
    pinDI_Input();
  }
  pinDI_Output();
  
}
//send start signal to TM1637
void TM1637::start(void)
{
  HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(DI_GPIO_Port, DI_Pin, GPIO_PIN_SET);	
  HAL_GPIO_WritePin(DI_GPIO_Port, DI_Pin, GPIO_PIN_RESET);	
  HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);	
} 
//End of transmission
void TM1637::stop(void)
{
  HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DI_GPIO_Port, DI_Pin, GPIO_PIN_RESET);	
  HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(DI_GPIO_Port, DI_Pin, GPIO_PIN_SET); 
}
//display function.Write to full-screen.
void TM1637::display(int8_t DispData[])
{
  int8_t SegData[4];
  uint8_t i;
  for(i = 0;i < 4;i ++)
  {
    SegData[i] = DispData[i];
  }
  coding(SegData);
  start();          //start signal sent to TM1637 from MCU
  writeByte(ADDR_AUTO);//
  stop();           //
  start();          //
  writeByte(Cmd_SetAddr);//
  for(i=0;i < 4;i ++)
  {
    writeByte(SegData[i]);        //
  }
  stop();           //
  start();          //
  writeByte(Cmd_DispCtrl);//
  stop();           //
}
//******************************************
void TM1637::display(uint8_t BitAddr,int8_t DispData)
{
  int8_t SegData;
  SegData = coding(DispData);
  start();          //start signal sent to TM1637 from MCU
  writeByte(ADDR_FIXED);//
  stop();           //
  start();          //
  writeByte(BitAddr|0xc0);//
  writeByte(SegData);//
  stop();            //
  start();          //
  writeByte(Cmd_DispCtrl);//
  stop();           //
}

void TM1637::clearDisplay(void)
{
  display(0x00,0x7f);
  display(0x01,0x7f);
  display(0x02,0x7f);
  display(0x03,0x7f);  
}
//To take effect the next time it displays.
void TM1637::set(uint8_t brightness,uint8_t SetData,uint8_t SetAddr)
{
  Cmd_SetData = SetData;
  Cmd_SetAddr = SetAddr;
  Cmd_DispCtrl = 0x88 + brightness;//Set the brightness and it takes effect the next time it displays.
}

//Whether to light the clock point ":".
//To take effect the next time it displays.
void TM1637::point(int PointFlag)
{
  _PointFlag = PointFlag;
}
void TM1637::coding(int8_t DispData[])
{
  uint8_t PointData;
  if(_PointFlag == POINT_ON)PointData = 0x80;
  else PointData = 0; 
  for(uint8_t i = 0;i < 4;i ++)
  {
    if(DispData[i] == 0x7f)DispData[i] = 0x00;
    else DispData[i] = TubeTab[DispData[i]] + PointData;
  }
}
int8_t TM1637::coding(int8_t DispData)
{
  uint8_t PointData;
  if(_PointFlag == POINT_ON)PointData = 0x80;
  else PointData = 0; 
  if(DispData == 0x7f) DispData = 0x00 + PointData;//The bit digital tube off
  else DispData = TubeTab[DispData] + PointData;
  return DispData;
}