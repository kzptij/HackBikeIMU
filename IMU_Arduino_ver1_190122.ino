/*
Arduino Uno
uno int(2byte):-32768, +32767  //(1byte):-128, +127
uno unsigned int(2byte): 0, 65535
*/
#include <NineAxesMotion.h>
#include <Wire.h>

NineAxesMotion mySensor;
bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream

void setup() {
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.

  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_IMUPLUS);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
}

void loop() {
  byte var = Serial.read();

  if (var = byte('R')) {

    if (updateSensorData)  //Keep the updating of data as a separate task
    {
      mySensor.updateQuat();
      mySensor.updateGyro();
      mySensor.updateAccel();
      mySensor.updateLinearAccel();
      mySensor.updateGravAccel();
      mySensor.updateCalibStatus();  //Update the Calibration Status
      updateSensorData = false;
    }

    /*クォータニオン算出*/
    int16_t w_data = mySensor.readQuatW();
    int16_t x_data = mySensor.readQuatX();
    int16_t y_data = mySensor.readQuatY();
    int16_t z_data = mySensor.readQuatZ();

    float w = (float)((float)w_data / 16384);
    float x = (float)((float)x_data / 16384);
    float y = (float)((float)y_data / 16384);
    float z = (float)((float)z_data / 16384);

    /*クォータニオンからroll, pith, yaw算出 deg*/
    float ysqr = y * y;
    // roll (x-axis rotation)
    float t0 = +2.0 * (w * x + y * z);
    float t1 = +1.0 - 2.0 * (x * x + ysqr);
    float roll = atan2(t0, t1);
    // pitch (y-axis rotation)
    float t2 = +2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    float pitch = asin(t2);
    // yaw (z-axis rotation)
    float t3 = +2.0 * (w * z + x * y);
    float t4 = +1.0 - 2.0 * (ysqr + z * z);
    float yaw = atan2(t3, t4);
    roll *= 57.2957795131;
    pitch *= 57.2957795131;
    yaw *= 57.2957795131;
    
    /* offset値：自転車へのセンサ取付物理的な位置補正 */
    /* ビビEXシティ= 前：roll 581, pitch 31、 後：roll -32,pitch 118  */
    /* ビビライフ　= 前：roll 140, pitch 128、後：roll -155,pitch 54  */
    /* オフタイム　= 前：roll 200, pitch 150、後：roll -86,pitch -766 */    
    int roll_i  = int(roll * 100 - 200);
    int pitch_i = int(pitch * 100 - 150);
    int yaw_i   = int(yaw * 100);

    /*Gyro算出 uT*/
    float gy_x = mySensor.readGyroX();
    float gy_y = mySensor.readGyroY();
    float gy_z = mySensor.readGyroZ();
    //送信データ変換
    int gy_x_i = int(gy_x * 100);
    int gy_y_i = int(gy_y * 100);
    int gy_z_i = int(gy_z * 100);

    /*加速度算出 m/s2 */
    float a_x = mySensor.readAccelX();
    float a_y = mySensor.readAccelY();
    float a_z = mySensor.readAccelZ();
    //送信データ変換
    int a_x_i = int(a_x * 100);
    int a_y_i = int(a_y * 100);
    int a_z_i = int(a_z * 100);

    /*並進加速度算出 m/s2*/
    float la_x = mySensor.readLinearAccelX();
    float la_y = mySensor.readLinearAccelY();
    float la_z = mySensor.readLinearAccelZ();
    //送信データ変換
    int la_x_i = int(la_x * 100);
    int la_y_i = int(la_y * 100);
    int la_z_i = int(la_z * 100);

    /*重力加速度算出 m/s2*/
    float ga_x = mySensor.readGravAccelX();
    float ga_y = mySensor.readGravAccelY();
    float ga_z = mySensor.readGravAccelZ();
    //送信データ変換
    int ga_x_i = int(ga_x * 100);
    int ga_y_i = int(ga_y * 100);
    int ga_z_i = int(ga_z * 100);


    /*データ送信 */
    const int val_size = 15;     //送信データ数
    int values[val_size] = {roll_i, pitch_i, yaw_i, gy_x_i, gy_y_i, gy_z_i, a_x_i, a_y_i, a_z_i, la_x_i, la_y_i, la_z_i, ga_x_i, ga_y_i, ga_z_i};

    Serial.write("H");  //ヘッダ送信　1byte(0-255)送信可

    for (int i = 0; i < val_size; i++) {
      uint8_t high = (uint8_t)((((unsigned int)(values[i] + 32768)) >> 8) & 0xFF);
      uint8_t low  = (uint8_t)((((unsigned int)(values[i] + 32768)) >> 0) & 0xFF);
      Serial.write(high);     //1byte(0-255)送信可
      Serial.write(low);      //1byte(0-255)送信可
    }
    Serial.flush();  //serial dataの送信が終わるまで待つ
    //delay(3);   //3ms

    updateSensorData = true;
    var = 0;
  }
}
