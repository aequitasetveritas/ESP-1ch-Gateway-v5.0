#include "lcd.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <Arduino.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Set the LCD I2C address, if it's not working try 0x27.

void lcd_init(void)
{
    // GPIO2 SDA , GPIO0 SCL
    Wire.begin(2, 0);
    lcd.begin(20, 4); // initialize the lcd
    lcd.home();       // go home
    lcd.print("Inito ");
    lcd.setCursor(0, 1); // go to the next line
    lcd.print(" OK   ");
}


void lcd_update(uint32_t rx, uint32_t ok, uint32_t f, uint8_t sf){

    static uint32_t pv;

    if((millis()-pv) > 1000){
        pv = millis();
    }else
    {
        {return;}
    }
    
    char prim_linea[21];
    char seg_linea[21];
    char cua_linea[21];


    snprintf(prim_linea,21,"rcv: %d ok: %d", rx, ok);
    snprintf(seg_linea,21,"Wifi OK - Srvr OK" );
    snprintf(cua_linea,21,"%d.%d Mhz SF%d",f/1000000, (f%(1000000)/100000), sf);

    lcd.home();
    lcd.print(prim_linea);
    lcd.setCursor(0,1);
    lcd.print(seg_linea);
    lcd.setCursor(0,3);
    lcd.print(cua_linea);
}


void lcd_line3(const char * line){
    lcd.setCursor(0,2);
    lcd.print(line);
}