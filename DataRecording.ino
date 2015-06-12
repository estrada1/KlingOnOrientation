/////////////////////////////////////////////////////////////
// General Helper Functions
// Useful for collecting data and spitting it back out
/////////////////////////////////////////////////////////////

// log_data()
// Circular log for data
void log_data() {
  
  //looping
  time[entry] = now;
  //xAccel[entry] = imu->getAccel().x();
  yAccel[entry] = imu->getAccel().y();
  magAccel[entry] = float(accmag); 
  buttonRelease[entry] = digitalRead(tailDownSwitch);
  entry = (entry+1)%MAX_SAMPLES;  

}


void dump_data() {

  Serial.println();
  Serial.println();
  Serial.println("Recorded Data: ");
  Serial.println("time, PWM, th_sf, buttonRelease");

  unsigned long timePrev = 0;
  for ( int i = 1 ; i < MAX_SAMPLES; i++) {
    int thisIndex = (i+entry)%MAX_SAMPLES;
    Serial.print(time[thisIndex]); Serial.print(", ");
    Serial.print(yAccel[thisIndex]); Serial.print(", ");
    Serial.print(magAccel[thisIndex]); Serial.print(", ");
    Serial.print(buttonRelease[thisIndex]); Serial.print(", ");
    Serial.println(time[thisIndex] - timePrev);
    timePrev = time[thisIndex];
  }
}

// For logging the first MAX_SAMPLES of something 
void log_data_front() {
  
  if(entry<=MAX_SAMPLES){
    //looping
    time[entry] = now;
    pwmRecord[entry] = float(pwm_tail);
    wRecord[entry] = float(w_gyro);
    //muRecord[entry] = float(mu);
    //yAccRecord[entry] = y_acc;
    //magAccel[entry] = float(accmag); 
    //yGyroRecord[entry] = float(y_gyro);
    angleRecord[entry] = float(th_sf);
    
    entry = (entry+1);  
  }

}

// Dumps the data for log_data_front where the index order is ocrrect
void dump_data_front() {

  Serial.println();
  Serial.println();
  Serial.println("Recorded Data: ");
  Serial.println("time, PWM, w_gyro, th_sf, delta");

  unsigned long timePrev = 0;
  for ( int thisIndex = 1 ; thisIndex < MAX_SAMPLES; thisIndex++) {
    Serial.print(time[thisIndex]); Serial.print(", ");
    Serial.print(pwmRecord[thisIndex]); Serial.print(", ");
    //Serial.print(muRecord[thisIndex]); Serial.print(", ");
    Serial.print(wRecord[thisIndex]); Serial.print(", ");
    //Serial.print(yGyroRecord[thisIndex]); Serial.print(", ");
    //Serial.print(yAccRecord[thisIndex]); Serial.print(", ");
    //Serial.print(magAccel[thisIndex]); Serial.print(", ");
    Serial.print(angleRecord[thisIndex]); Serial.print(", ");
    Serial.println(time[thisIndex] - timePrev);
    timePrev = time[thisIndex];
  }
}

void print_everything() {

  //      Serial.println("Measurements: th_sf, g_accx, g_accy, w_gyro");
  //      Serial.print(now); Serial.print(",");
  //      Serial.print(th_sf); Serial.print(", ");
  //      Serial.print(mu); Serial.print(", ");
  //      Serial.print(wc); Serial.print(", ");
  //      Serial.print(g_accx); Serial.print(", ");
  //      Serial.print(g_accy); Serial.print(", ");
  //      Serial.println(w_gyro);
  //      Serial.println("Fuzzy Logic: x1 x2 x3 x4");
  //      Serial.print(xb1); Serial.print(",");
  //      Serial.print(xb2); Serial.print(",");
  //      Serial.print(xb3); Serial.print(",");
  //      Serial.print(xb4); Serial.print(",");
  //      Serial.print(ddtaccmag); Serial.print(", ");
  //      Serial.print(ddtg_accx); Serial.print(", ");
  //      Serial.print(ddtg_accy); Serial.print(", ");
  //      Serial.print("accmag: "); Serial.println(accmag);

}




