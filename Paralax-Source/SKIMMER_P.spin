{{
*****************************************
* Servo32_v5 Demo v1.5                  *
* Author: Beau Schwabe                  *
* Copyright (c) 2009 Parallax           *
* See end of file for terms of use.     *
*****************************************
}}

CON
    _clkmode = xtal1 + pll16x                           
    _xinfreq = 5_000_000                                'Note Clock Speed for your setup!!

    ServoCh1 = 0                                        'Select DEMO servo

VAR

 byte   LMTR[11]
 long   ILMTR
 byte   RMTR[11]
 long ir
 long il
 long prevEnc[2]

    
OBJ
  SERVO :       "Servo32v5.spin"
  HB25  :       "CJ_HB25_014.spin"
  PST   :       "Parallax Serial Terminal.spin"
  SER   :       "FullDuplexSerial.spin"
  MOTOR :       "PPC_DriverV1.4.spin"
  PID   :       "PID V3.spin"
    
PUB Main | i, j,temp

  HB25.config(18,1,1)
  PST.start(115200)
  MOTOR.start(16)
  
   i:= 0
   j:= 0

   ir:= 0
   il:= 0
   
   repeat
      waitcnt(clkfreq/5+ cnt)   'wait
     

      ir := 1400
      il := 1400                 

      HB25.set_motor1(il)
      HB25.set_motor2(2976-ir)
 
      PST.char("$")
      PST.dec(MOTOR.Position(0)- prevEnc[0])
      PST.char(";")
      PST.dec(MOTOR.Position(1)- prevEnc[1])
      PST.char("#")

      prevEnc[0] := MOTOR.Position(0)
      prevEnc[1] := MOTOR.Position(1)
        
      i:= 0
      j:= 0
      
      


PUB StrToDec(stringptr,terminator) : value | char, index, multiply

value := index := 0
repeat until ((char:= byte[stringptr][index++]) == terminator)
  if char => "0" and char =< "9"
    value := value * 10 + (char - "0")
  if byte[stringptr] == "-"
    value := - value