   
#ifdef ARTVA_LOG
  //SD Variables
  String dataString = "";
  String fileName = "global.txt";
  boolean gpsInitFlag = false; //Comincio a scrivere quando ho inizializzato il GPS (provo anche a prendermi la data e ora...)
  char fnString[20];
  
  void InitSDCard(){
    //SD INIT
    Serial.print("Initializing SD card...");
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(10, OUTPUT);
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      // don't do anything more:
      return;
    }
    Serial.println("card initialized.");
    //END OF SD INIT
  }
#endif

#ifdef USE_ARTVA

  #define INTERVAL 500  //ms loop
  #define ARTVAINTERVAL 0x01
  int artvaDiv = 0;
  char tdbuf[64];
  uint32_t lastPrint = 0;
  /////////////LOOP/////////////
  void ArtvaGPSCheckNewData(){
    
    uint32_t now=millis();   //Number of milliseconds since the program started 
#ifdef ARTVA_LOG    
    if(SerialGPS.available()){ // check for gps data 
      char c = SerialGPS.read();
      gps1.encode(c);
      //Serial.print(c);
    }
#endif
    while (SerialARTVA.available()>0){
      byte b = SerialARTVA.read()-0x55;
      // SerialDEBUG.print("ARTVA BYTE:");SerialDEBUG.println(b);
      
      switch (framebytes)
      {
          case 0:
              if (b == 0xDE)
              {
                  framebuffer[framebytes] = b;
                  framebytes++;
              }
              else
              {
                  //Clear framebuffer?
                  framebytes = 0;
              }
              break;
          case 1:
              if (b == 0xFA)
              {
                  framebuffer[framebytes] = b;
                  framebytes++;
              }
              else
              {
                  //Clear framebuffer?
                  framebytes = 0;
              }
              break;
          case 2:
              if (b == 0xBE)
              {
                  framebuffer[framebytes] = b;
                  framebytes++;
              }
              else
              {
                  //Clear framebuffer?
                  framebytes = 0;
              }
              break;
          case 3:
              if (b == 0xBA)
              {
                  framebuffer[framebytes] = b;
                  framebytes++;
              }
              else
              {
                  //Clear framebuffer?
                  framebytes = 0;
              }
              break;
          /*
          case 28:
              break;
          case 29:
              break;
          case 30:
              break;
          case 31:
              break;
          */
          default:
              if (framebytes<32)
              {
                  framebuffer[framebytes] = b;
                  framebytes++;
              }
              if (framebytes >= 32)
              {
                  //Clear framebuffer?
                  memcpy(artvaframe,framebuffer,32);
                  //artvavalid=true;
                  parseARTVAFrame();
                  framebytes = 0;
                  artvaDiv++;
                  if (artvaDiv & ARTVAINTERVAL){
                    //Re-inoltro a terra
                    // SerialDEBUG.println("SENDARTVA"); // Commented BY NIcola 2016/09/20
                    // SerialGPS.write(artvaframe,32);  // Commented BY NIcola 2016/09/20
                    //SerialXBEE.println("PROVA!");
                    artvaDiv = 0;
                  }
              }
              break;
      }
    }
  
    if (now - lastPrint > INTERVAL){
      flagArtvaInterval = true;
#ifdef ARTVA_LOG
      Serial.print(now);
      dataString = ""; //Log string
    
      if (!gpsInitFlag && gps1.location.isValid() && gps1.date.isValid()){
        gpsInitFlag = true;
        sprintf(tdbuf, "%02d%02d%02d%02d.txt", gps1.date.month(),gps1.date.day(),gps1.time.hour(),gps1.time.minute());
        //sprintf(tdbuf, "L%02d%02d%02d.txt", gps1.date.day(),gps1.time.hour(),gps1.time.minute());
        //fileName = +++".txt"; //Uso il tempo GPS (se riesco)
        //fileName.toCharArray(fnString,12);
        strcpy(fnString, tdbuf);
        Serial.print("Init File: ");
        Serial.println(fnString);
      }
      if (gpsInitFlag){
        lat1=gps1.location.lat();
        lon1=gps1.location.lng();
        // print latitude 1
        Serial.print("   ");Serial.print(lat1, 8);
        // print longitude 1
        Serial.print("   ");Serial.print(lon1, 8); 
        // print number of sat
        Serial.print("   ");Serial.print(gps1.satellites.value());
        //Stringa da loggare
        sprintf(tdbuf, "%02d", gps1.time.hour());
        dataString += tdbuf;
        dataString += ";";
        sprintf(tdbuf, "%02d", gps1.time.minute());
        dataString += tdbuf;
        dataString += ";";
        sprintf(tdbuf, "%02d", gps1.time.second());
        dataString += tdbuf;
        dataString += ";";
        sprintf(tdbuf, "%02d", gps1.time.centisecond());
        dataString += tdbuf;
        dataString += ";";
        sprintf(tdbuf, "%0.8f", lat1);
        dataString += tdbuf;
        dataString += ";";
        sprintf(tdbuf, "%0.8f", lon1);
        dataString += tdbuf;
        dataString += ";";
        dataString += gps1.satellites.value();
        dataString += ";";
        dataString += _distanceOne;
        dataString += ";";
        dataString += _angleOne;
        dataString += ";";
        dataString += _distanceTwo;
        dataString += ";";
        dataString += _angleTwo;
        dataString += ";";
        dataString += _distanceThree;
        dataString += ";";
        dataString += _angleThree;
        dataString += ";";
        dataString += _distanceFour;
        dataString += ";";
        dataString += _angleFour;
        
        Serial.println();
        //Quando ho costruito la mia stringa, vado di scrittura
        File dataFile = SD.open(fnString, FILE_WRITE);
        if (dataFile) {
          dataFile.println(dataString);
          dataFile.close();
          // print to the serial port too:
          Serial.println(dataString);
        }
        // if the file isn't open, pop up an error:
        else {
          Serial.println("error opening log file!");
        }
      }
      
      //Invio su schermo
      //SerialXBEE.print("S ");
      //SerialXBEE.print(_distanceOne);
      //SerialXBEE.print(" ");
#endif
      //reset time    
      lastPrint = now;
    }
    
  }
#endif

