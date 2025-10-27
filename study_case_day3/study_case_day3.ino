/*
   Nama  : Christian Riovaldo Labina
  NIM   : 24/533483/PA/22581

  Program ini bertujuan untuk membaca data rotasi roll, pitch, dan yaw dari sensor MPU6050.
  Data tersebut kemudian digunakan untuk mengontrol posisi 5 motor servo. Pada sistem ini
  juga menggunakan sensor PIR untuk mendeteksi adanya gerakan eksternal.

  Library yang digunakan:
  - Wire.h = Untuk mengatur komunikasi I2C antara mikrokontroller ESP32 dan sensor MPU6050.
  - Adafruit_MPU6050.h = Library resmi dari Adafruit untuk berkomunikasi dengan sensor MPU6050.
  - Adafruit_Sensor.h = Untuk standarisasi data dari berbagai sensor yang kompatibel dengan Adafruit.
  - ESP32Servo.h = Menyediakan fungsi kontrol PWM untuk memudahkan kontrol posisi servo pada ESP32.
*/

#include <Wire.h> 
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

#define PIR_PIN 26      // Pin sensor PIR
#define SDA_PIN 21      // Pin SDA I2C MPU6050
#define SCL_PIN 22      // PIN SCL I2C MPU6050

#define SERVO1_PIN 19   // Pin servo 1
#define SERVO2_PIN 18   // Pin servo 2
#define SERVO3_PIN 17   // Pin servo 3
#define SERVO4_PIN 16   // Pin servo 4
#define SERVO5_PIN 4    // Pin servo 5

Adafruit_MPU6050 mpu;   // Membuat variabel objek sensor MPU6050
Servo servo1, servo2, servo3, servo4, servo5;   // Membuat variabel objek servo 1-5

float roll = 0, pitch = 0, yaw = 0, currentYaw = 0;  // Deklarasi awal orientasi roll, pitch, dan yaw
unsigned long lastTime = 0;                          // Deklarasi waktu loop sebelumnya untuk perhitungan elapsed time
int initialPos = 90;                                 // Deklarasi posisi awal servo

void setup() {
  Serial.begin(115200);           // Set up komunikasi serial    
  Wire.begin(SDA_PIN, SCL_PIN);   // Set up I2C
  mpu.begin();                    // Set up sensor MPU 6050 

  /*
    Set up sensitivitas sensor MPU6050:
    - mpu.setGyroRange(MPU6050_RANGE_500_DEG), mengatur rentang pengukuran giroskop yang berarti sensor dapat mengukur
      kecepatan rotasi dari -500 hingga +500 derajat per detik pada sumbu x, y, dan z.
    - mpu.setAccelerometerRange(MPU6050_RANGE_4_G), mengatur rentang pengukuran akselerometer yang berarti sensor dapat
      mengukur percepatan hingga 4 kali gaya gravitasi pada sumbu x, y, dan z.
    - mpu.setFilterBandwidth(MPU6050_BAND_21_HZ), mengatur lebar bandwith (pita) pada filter yang berarti sensor hanya
      akan merespon perubahan gerakan dan rotasi yang terjadi hingga pada frekuensi 21Hz.
  */
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Inisialisasi pin tiap servo
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);

  // Set up posisi awal tiap servo
  servo1.write(initialPos);
  servo2.write(initialPos);
  servo3.write(initialPos);
  servo4.write(initialPos);
  servo5.write(initialPos);

  pinMode(PIR_PIN, INPUT);    // Set up sensor PIR sebagai input
  lastTime = millis();        // Menyimpan waktu awal
}

void loop() {
  sensors_event_t a, g, temp;   // Struktur data untuk menyimpan data akselerometer (a), giroskop (g), dan temperature (temp)
  mpu.getEvent(&a, &g, &temp);  // Membaca data akselerometer, giroskop, dan temperature

  // Menghitung elapsed time
  unsigned long currentTime = millis();
  float dt = (currentTime-lastTime)/1000.0;
  lastTime = currentTime;

  // Menghitung perubahan sudut berdasarkan data giroskop pada sumbu x, y, dan z
  roll += g.gyro.x*dt*180/PI;
  pitch += g.gyro.y*dt*180/PI;
  yaw = g.gyro.z*180/PI;

  /* 
    - Menghitung sudut dari akselerometer untuk roll dan pitch.
    - Yaw tidak bisa dihitung karena rotasinya terhadap sumbu z, arah gravitasinya tetap sama sehingga akselerometer
      tidak bisa mendeteksi yaw.
  */
  float accRoll = atan2(a.acceleration.y, a.acceleration.z)*180/PI;
  float accPitch = atan2(-a.acceleration.x, sqrt(pow(a.acceleration.y, 2)+pow(a.acceleration.z, 2)))*180/PI;

  /*
    Filter komplementer: Menggabungkan data dari akselerometer dan giroskop
    untuk menghasilkan estimasi orientasi yang lebih akurat.
  */
  roll = 0.98*roll + 0.02*accRoll;
  pitch = 0.98*pitch + 0.02*accPitch;

  // Menampilkan hasil roll, pitch, dan yaw di Serial Monitor
  Serial.print("Roll: "); Serial.print(roll, 2);
  Serial.print(" | Pitch: "); Serial.print(pitch, 2);
  Serial.print(" | Yaw: "); Serial.println(yaw, 2);

  rotateRoll();   // Menggerakkan servo 1 dan servo 2
  rotatePitch();  // Menggerakkan servo 3 dan servo 4
  rotateYaw();    // Menggerakkan servo 5

  /*
    Mendeteksi gerakan eksternal yang terjadi menggunakan sensor PIR. Jika terdapat gerakan eksternal,
    semua servo akan berputar acak ke posisi tertentu, kemudian akan kembali ke posisi awal.
  */
  int pirState = digitalRead(PIR_PIN);
  if (pirState == HIGH) {
    servo1.write(30);
    servo2.write(150);
    servo3.write(60);
    servo4.write(120);
    servo5.write(45);
    delay(1000);
    servo1.write(initialPos);
    servo2.write(initialPos);
    servo3.write(initialPos);
    servo4.write(initialPos);
    servo5.write(initialPos);
  }

  delay(20);
}

/*
  Fungsi yang menggerakkan servo berdasarkan roll, pitch, dan yaw:
  - rotateRoll() = servo 1 dan servo 2
  - rotatePitch() = servo 3 dan servo 4
  - rotateYaw() = servo 5

  map(value, fromLow, fromHigh, toLow, toHigh): 
  - Berfungsi untuk mengubah suatu nilai rentang servo ke nilai rentang lainnya
  - value = nilai yang akan dikonversi
  - fromLow = batas bawah rentang asal
  - fromHigh = batas atas rentang asal
  - toLow = batas bawah rentang baru
  - toHigh = batas atas rentang baru

  constrain(value, bottom, top):
  - Berfungsi untuk membatasi nilai agar berada dalam rentang tertentu
  - value = nilai yang akan dibatasi
  - bottom = batas bawah
  - top = batas atas
*/

void rotateRoll() {
  int rollAngle = map((int)roll,-90,90,0,180);  // Mengubah rentang -90 hingga 90 derajat menjadi 0-180 derajat
  rollAngle = constrain(rollAngle,0,180);       // Membatasi sudut rotasi roll 0-180 derajat
  servo1.write(180-rollAngle);                  // Servo 1 digerakkan berlawanan arah rotasi roll
  servo2.write(180-rollAngle);                  // Servo 2 digerakkan berlawanan arah rotasi roll
}

void rotatePitch() {
  int pitchAngle = map((int)pitch,-90,90,0,180);  // Mengubah rentang -90 hingga 90 derajat menjadi 0-180 derajat
  pitchAngle = constrain(pitchAngle,0,180);       // Membatasi sudut rotasi pitch 0-180 derajat
  servo3.write(pitchAngle);                       // Servo 3 digerakkan searah dengan rotasi pitch
  servo4.write(pitchAngle);                       // Servo 4 digerakkan searah dengan rotasi pitch
}

void rotateYaw() {
  /*
    Servo 5 hanya akan digerkakkan jika nilai yaw saat ini berbeda dengan nilai yaw sebelumnya. Hal tersebut dibuat
    karena servo 5 harus kembali ke posisi semula setelah 1 detik menerima perintah untuk bergerak sesuai dengan arah
    yaw, sehingga servo 5 tidak perlu mempertahankan posisinya.
  */
  if (currentYaw != yaw) {
    int yawAngle = map((int)yaw,-180,180,0,180);  // Mengubah rentang -180 hingga 180 derajat menjadi 0-180 derajat
    yawAngle = constrain(yawAngle,0,180);         // Membatasi sudut rotasi yaw 0-180 derajat
    servo5.write(yawAngle);                       // Servo 5 digerakkan searah dengan rotasi yaw
    currentYaw = yaw;                             // Menyimpan nilai yaw terakhir yang dikirim ke servo 5
  }
  
  /*
    Memberi delay 1 detik setelah servo 5 bergerak, lalu kembali ke posisi awal.
    Delay diatur dengan mempertimbangkan elapsed time dengan millis() agar tidak
    mempengaruhi bacaan dari sensor. Delay biasa dengan perintah delay() bersifat
    blocking sehingga bacaan sensor dan perintah lain juga akan berhenti tereksekusi,
    mempengaruhi perhitungan rotasi roll, pitch, dan yaw.
  */
  static unsigned long yawTimeNow = 0;  // Static berarti variabel hanya diinisialisasi sekali 
  if (millis()-yawTimeNow > 1000) {
    servo5.write(initialPos);
    yawTimeNow = millis();
  }
}