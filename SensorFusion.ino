
// inputs: gyro signals, acc signals, ITG3200_OFF X
// outputs: th_sf

void sensorfusion() {

  //dt = double(dt_l)/1000;


  //w_gyro = (sign) * ( gyro[-] + offset) * gain, sign from mounting, offset read from WAITBEGIN tab, gain = totalrange/itg3200resolution = 4000/2^15
  //w_gyro = -(gyro[0]-ITG3200_OFF_X)*0.1221;
  w_gyro = -(imu->getGyro().z())*180/pi;

  //scaled accelerometer signals, found acc[-] max and min by hand
  //g_acc- = (acc[-]+offset)*gain, gain = 1/((|max|+|min|)/2), offset = -(|max|-|min|)/2
  //g_accz = (acc[2]+20)*0.004; // max = 230, min = -270
  //g_accy = (acc[1]-7.5)*0.0038; // max = 270, min = -255
  g_accx = imu->getAccel().x();
  g_accy = imu->getAccel().y();
  g_accz = imu->getAccel().z();

  // theta acc = atan2(b1,b2), b1 wrt to horizontal is theta, b2 is perp to b1. [deg]
  //th_acc = atan2(-g_accy,g_accz)*180/3.14159;
  th_acc = atan2(-g_accx, g_accy) * 180 / 3.14159;

  // calc TV wc
  ddtg_accx = (g_accx - g_accxp) / dt;
  ddtg_accy = (g_accy - g_accyp) / dt;
  accmag = sqrt(pow(g_accx, 2) + pow(g_accy, 2));
  accmag3D = sqrt(pow(g_accx, 2) + pow(g_accy, 2)+pow(g_accz,2) );
  ddtaccmag = sqrt(pow(ddtg_accx, 2) + pow(ddtg_accy, 2));
  ddtw_gyro = (w_gyro - w_gyrop) / dt;
  xb1 = 0.5 * (1 + tanh(s1 * (abs(accmag - 1) - xo1)));
  xb2 = 0.5 * (1 + tanh(s2 * (ddtaccmag - xo2)));
  xb3 = 0.5 * (1 + tanh(s3 * (abs(w_gyro) - xo3)));
  xb4 = 0.5 * (1 + tanh(s4 * (abs(ddtw_gyro) - xo4)));
  mu = (1 - xb1) * (1 - xb2) * (1 - xb3) * (1 - xb4); // fuzzy logic
  wc = mu * whigh + (1 - mu) * wlow;           // dynamic cut off freq

  // sensor fusion
  y_acc = 1 / (2 + wc * dt) * (-(-2 + wc * dt) * y_accp + wc * dt * th_acc + wc * dt * th_accp); // C2D tustin of wc/(s+wc), low pass
  y_gyro =  1 / (2 + wc * dt) * (-(-2 + wc * dt) * y_gyrop + dt * w_gyro + dt * w_gyrop); // C2D tustin of 1/s * s/(s+wc) = 1/(s+wc), integration then high pass
  th_sf = y_acc + y_gyro; // add both signals for sensor fusion estimate

  // store previous global variables
  y_gyrop = y_gyro;
  y_accp = y_acc;
  th_accp = th_acc;
  w_gyrop = w_gyro;
  g_accxp = g_accx;
  g_accyp = g_accy;

}

