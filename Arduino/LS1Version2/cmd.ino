#define CMD(a,b) ( a + (b << 8))
#define TRUE 1
#define FALSE 0

int ProcCmd(char *pCmd)
{
	short *pCV;
	short n;
	long lv1;
	char s[22];
        unsigned int tday;
        unsigned int tmonth;
        unsigned int tyear;
        unsigned int thour;
        unsigned int tmin;
        unsigned int tsec;

	pCV = (short*)pCmd;

	n = strlen(pCmd);
	if(n<2) 
          return TRUE;

	switch(*pCV)
	{                           
      case ('R' + ('D'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        rec_dur = lv1;
        break;
      }
      
      case ('R' + ('I'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        rec_int = lv1;
        break;
      } 

      case ('S' + ('G'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        gainSetting = lv1;
        if((gainSetting<0) | (gainSetting>15)) gainSetting = 4;
        EEPROM.write(15, gainSetting); //byte
        break;
      }
      
      case ('N' + ('D'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        noDC = lv1;
        break;
      }
	}	
	return TRUE;
}

boolean LoadScript()
{
  char s[30];
  char c;
  short i;

#if USE_SDFS==1
  FsFile file;
#else
  File file;
#endif
  unsigned long TM_byte;
  int comment_TM = 0;

  // Read card setup.txt file to set date and time, recording interval
  sd.chdir(); // only to be sure to star from root
  file=sd.open("setup.txt");
  if(file)
  {
    do{
      	i = 0;
      	s[i] = 0;
        do{
            c = file.read();
	          if(c!='\r') s[i++] = c;
            if(c=='T') 
            {
              TM_byte = file.position() - 1;
              comment_TM = 1;
            }
            if(i>29) break;
	        }while(c!='\n');
      	  s[--i] = 0;
          if(s[0] != '/' && i>1)
          {
            ProcCmd(s);
      	  }
      }while(file.available());
      file.close();  

      
      // comment out TM line if it exists
      if (comment_TM)
      {
        Serial.print("Comment TM ");
        Serial.println(TM_byte);
        
        sd.chdir(); // only to be sure to star from root
        file = sd.open("setup.txt", FILE_WRITE); // WMXZ check mod
        file.seek(TM_byte);
        file.print("//");
        file.close();

      }
      
  }
  else
  {   
    Serial.println("setup.txt not opened");
    display.println("no setup file");
    return 0;
  }
 return 1;	
}
