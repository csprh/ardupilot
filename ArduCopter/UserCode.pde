/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()

{
  pinMode(AN4, INPUT);  //59
  
  pinMode(AN2, OUTPUT); //60
  pinMode(AN5, OUTPUT); //60


  //digitalWrite(AN5,0);
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
  int p;
  static int thisTime = 0;
  static int startTimer = 0;
  if (startTimer == 1) {thisTime++;}
  p = digitalRead(AN4);
  
  // initialise logger (by arming and making sure it doesn't disarm
  if ((p == HIGH)&&(startTimer!=1)) {
    startTimer = 1;
    init_arm_motors();
    digitalWrite(AN2,LOW);
    digitalWrite(AN5,LOW);
    set_pre_arm_check(true);
    set_pre_arm_rc_check(true);
  }
  

  if (thisTime == 20) {
    digitalWrite(AN2,HIGH);	  	
  }    
  // Start camera
  if (thisTime == 24) {
    digitalWrite(AN2,LOW);	  	
  }    
    
  // Switch on LED and log 
  if (thisTime == 30) {
    Log_Write_Camera2();
    digitalWrite(AN5,HIGH);
  }
  if (thisTime == 130) {
    Log_Write_Camera2();
    digitalWrite(AN5,LOW);
  }
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
