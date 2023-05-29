//SPI speed: MHz(real), kHz(Simulation)
//Slow_01: f = 20kHz => T = 50us (max)
//Slow_02: f = 1kHz => T = 1000us
//Slow_03: f = 2MHz => T = 0.5us
#define TFull 50
#define THalf TFull/2

#define SCK_PIN 4
#define MOSI_PIN 5
#define MISO_PIN 6
#define SS_PIN 7

/////dinh nghia chan do la chan input hoac chan output
//--------------
#define SCK_OUTPUT DDRD |= (1 << DDD4) //tac dong len chan so 4 cua portD
#define MOSI_OUTPUT DDRD |= (1 << DDD5)
#define MISO_INPUT DDRD &= ~(1 << DDD6)
#define SS_OUTPUT DDRD |= (1 << DDD7)

/////neu chan do la chan output thi cho chan do la 0 hoac 1, neu chan do la chan input thi doc gia tri 0 hoac 1 o chan do
//--------------
//#define MOSI_HIGH PORTD |= (1<<PD5) //00100000  //digitalWrite(MOSI, HIGH)
//#define MOSI_HIGH PORTD &= ~(1<<PD5) //11011111 //digitalWrite(MOSI, LOW)
//a = (value) ? x : y; //if(value == true) a=x; else a=y;
//gop lai
#define write_MOSI(x) PORTD = ((x)? (PORTD | (1 << PD5)) : (PORTD & ~(1 << PD5))) //volatile ep kieu chong viec code toi uu lam mat dong lenh
#define write_SS(x)   PORTD = ((x)? (PORTD | (1 << PD7)) : (PORTD & ~(1 << PD7)))
#define write_SCK(x)  PORTD = ((x)? (PORTD | (1 << PD4)) : (PORTD & ~(1 << PD4)))
#define read_MISO()   ((PIND & (1 << PIND6)) ? HIGH:LOW) //digitalRead(MISO_PIN)

void setup() {
  // put your setup code here, to run once:
  SPI_setup();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t rev[3];//recieve
  SPI_begin();
  rev[0] = SPI_transfer('1'); 
  rev[1] = SPI_transfer('2'); 
  rev[2] = SPI_transfer('3'); 
  SPI_end();
  Serial.println(rev[0]);
  Serial.println(rev[1]);
  Serial.println(rev[2]);
  Serial.println("M:" + String((char)rev[0]));
  Serial.println("M:" + String((char)rev[1]));
  Serial.println("M:" + String((char)rev[2]));
  delay(1000); //the transmition will reset after 1 second 
  
}

void SPI_setup()
{
  MOSI_OUTPUT;
  MISO_INPUT;
  SCK_OUTPUT;
  SS_OUTPUT;
  write_SCK(LOW);
  write_SS(HIGH); //mac dinh slave select o muc cao
  delay(1);
}

void SPI_begin()
{
  write_SS(LOW);
}

void SPI_end()
{
  write_SS(HIGH);
  write_SCK(LOW);
}

//MODE0: CPOL = 0, CPHASE = 0, bitOrder = MSB.
//byte_out = 0x55 (0101 0101)
//step1(7) = byte_out & 0x80 = 0101 0101 & 1000 0000 = 0000.0000 //giu lai gia tri cua bit cao nhat
//step2(6) = byte_out & 0x40 = 0101 0101 & 0100 0000 = 0100.0000
uint8_t SPI_transfer(uint8_t byte_out)
{
  uint8_t byte_in = 0; //0000.0000
  uint8_t ibit, res; 
  //ibit: 1000.0000 (0x80) -> 0100.0000 (0x40) ... -> 0000.0001 -> 0000.0000 (stop)

  for(ibit = 0x80; ibit > 0; ibit = ibit >> 1)
  {
    res = byte_out & ibit; //(#0=true), (=0=false)
    write_MOSI(res);

    delayMicroseconds(THalf);
    write_SCK(HIGH);

    if(read_MISO() == HIGH)
    {
      byte_in = byte_in | ibit; // 0000.0000 | 1000.0000 = 1000.0000
    }
//    else
//    {
//      byte_in = byte_in | ibit; //khong can thiet vi ban dau da cho byte_in = 0000.0000
//    }
    delayMicroseconds(THalf);
    write_SCK(LOW); //end 1 clock cycle 
  }
  return byte_in;
}
