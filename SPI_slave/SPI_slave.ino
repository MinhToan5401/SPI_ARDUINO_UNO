#define SCK_PIN 4
#define MOSI_PIN 5
#define MISO_PIN 6
#define SS_PIN 7

#define SCK_INPUT DDRD &= ~(1 << DDD4) //tac dong len chan so 4 cua portD
#define MOSI_INPUT DDRD &= ~(1 << DDD5)
#define MISO_OUTPUT DDRD |= (1 << DDD6)
#define SS_INPUT DDRD &= ~(1 << DDD7)

#define read_MOSI() ((PIND & (1 << PIND5)) ? HIGH:LOW) //digitalRead(MISO_PIN)
#define read_SS()   ((PIND & (1 << PIND7)) ? HIGH:LOW) //digitalRead(MISO_PIN)
#define read_SCK()  ((PIND & (1 << PIND4)) ? HIGH:LOW) //digitalRead(MISO_PIN)
#define write_MISO(x)   PORTD = ((x)? (PORTD | (1 << PD6)) : (PORTD & ~(1 << PD6)))

void setup() {
  // put your setup code here, to run once:
  SPI_setup();
  Serial.begin(9600);
}

void loop() {
  uint8_t rev[3];
  rev[0] = SPI_transfer('A'); //0100.0001
  rev[1] = SPI_transfer('B'); 
  rev[2] = SPI_transfer('C'); 
  Serial.println(rev[0]);
  Serial.println(rev[1]);
  Serial.println(rev[2]);
  Serial.println("S:" + String((char)rev[0]));
  Serial.println("S:" + String((char)rev[1]));
  Serial.println("S:" + String((char)rev[2]));
}

void SPI_setup(void)
{
  SCK_INPUT;
  MOSI_INPUT;
  MISO_OUTPUT;
  SS_INPUT;
}

//doc gui data dien ra dong thoi, song cong (full duplex)
//transfer data dung duoi goc do slave
uint8_t SPI_transfer(uint8_t byte_out)
{
  uint8_t byte_in = 0; //0000.0000
  uint8_t ibit, res; 
  while(read_SS() == HIGH); //waiting until SS = 0 (LOW), "Start Condition"
   for(ibit = 0x80; ibit > 0; ibit = ibit >> 1)
   {
    res = byte_out & ibit; //(#0=true), (=0=false)
    write_MISO(res);
    while(read_SCK() == LOW); //waiting until SCK = 1
    if(read_MOSI() == HIGH)
    {
      byte_in = byte_in | ibit;
    }
    while(read_SCK() == HIGH);//waiting until SCK = 0
    //until here, this is the "end of 1 clock cycle"
   }
   return byte_in;
}
