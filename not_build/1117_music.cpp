#include <Adafruit_LIS3DH.h>

#define LIS3DH_ADDRESS 0x19
Adafruit_LIS3DH accel = Adafruit_LIS3DH ();

int usePinCount = 8 ;
int usePin[] = { D0, D4, D6, D7, D8, D10, D11, D12 } ; // Leafony接続時は1行下を有効にする
    //int usePin[] = { D0, D1, D2, D3, D4, D5, D6 } ;
int DO = 0 ;
int RE = 1 ;
int MI = 2 ;
int FA = 3 ;
int SO = 4 ;
int RA = 5 ;
int SI = 6 ;
int DO2 = 7 ;
int NO = 8 ;
int END = 9 ;
int music[][256] = {
    //4つの音符 
    { DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, FA, 6, MI, 6, RE, 6, 
      DO, 6, RE, 6, MI, 6, FA, 6, FA, 12, 
      DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, FA, 6, MI, 6, RE, 6, 
      DO, 6, RE, 6, MI, 6, FA, 6, FA, 18, END },//坂道を登るときUphill A
    { RE, 6, MI, 6, FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, 
      FA, 6, MI, 6, RE, 6, MI, 6, FA, 12, 
      MI, 6, FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, 
      MI, 6, FA, 6, MI, 6, RE, 6, FA, 18, END },//坂道を登るときUphill B
    { FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, 
      FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 12, 
      MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, RE, 6, 
      DO, 6, RE, 6, MI, 6, FA, 6, MI, 12, END },//坂道を軽快に下るUphill A  
    { MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, RE, 6,
      DO, 6, RE, 6, MI, 6, FA, 6, MI, 12,
      FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, 
      RE, 6, DO, 6, RE, 6, MI, 6, FA, 18, END },//坂道を軽快に下るUphill B
    { DO, 6, NO, 6, RE, 6, MI, 6, FA, 6, MI, 6, RE, 6,
      DO, 6, NO, 6, MI, 6, NO, 6, FA, 6, NO, 6, MI, 6, NO, 6, 
      DO, 6, RE, 6, NO, 6, MI, 6, FA, 6, NO, 6, RE, 6, NO, 6, 
      DO, 6, MI, 6, FA, 12, END },//でこぼこ道を走るUphill A
    { MI, 6, NO, 6, FA, 6, RE, 6, DO, 6, NO, 6, RE, 6, MI, 6,
      FA, 6, NO, 6, MI, 6, RE, 6, DO, 6, NO, 6, RE, 6, NO, 6, 
      MI, 6, NO, 6, FA, 6, RE, 6, DO, 6, NO, 6, RE, 6, MI, 6,
      FA, 6, NO, 6, RE, 6, NO, 6, DO, 6, MI, 6, FA, 12, END },
      //でこぼこ道を走るUphill B 

    //9の音符  
    { DO, 6, RE, 6, MI, 6, FA, 6, SO, 6, SO, 6, FA, 6, MI, 6, 
      RE, 6, MI, 6, FA, 6, SO, 6, RA, 6, SI, 6, RA, 6, SO, 6, 
      FA, 6, SO, 6, RA, 6, SO, 6, FA, 6, MI, 6, RE, 6, MI, 6, 
      FA, 6, SO, 6, RA, 6, SI, 6, DO2, 12, END },//坂道を登るときUphill A 
    { RE, 6, MI, 6, FA, 6, SO, 6, RA, 6, SI, 6, RA, 6, SO, 6,
      FA, 6, MI, 6, RE, 6, MI, 6, FA, 6, SO, 6, RA, 6, SI, 6, 
      DO2, 6, SI, 6, RA, 6, SO, 6, FA, 6, MI, 6, RE, 6, MI, 6, 
      FA, 6, SO, 6, RA, 6, SI, 6, DO2, 12, END },//坂道を登るときUphill B 
    { SI, 6, RA, 6, SO, 6, FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, 
      MI, 6, FA, 6, SO, 6, RA, 6, SO, 6, FA, 6, MI, 12, MI, 6, 
      SO, 6, RA, 6, SO, 6, FA, 6, MI, 6, RE, 6, MI, 6, FA, 6, 
      SO, 6, RA, 6, SI, 6, RA, 6, SO, 6, FA, 12, END },//坂道を軽快に下るUphill A 
    { RA, 6, SO, 6, FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, 
      FA, 6, SO, 6, RA, 6, SO, 6, FA, 6, MI, 6, RE, 12, MI, 6, 
      FA, 6, SO, 6, RA, 6, SI, 6, RA, 6, SO, 6, FA, 6, MI, 12, 
      NO, 6, SO, 6, RA, 6, END },//坂道を軽快に下るUphill B 
    { DO, 6, NO, 6, MI, 6, FA, 6, SO, 6, FA, 6, MI, 6, DO, 6,
      NO, 6, SO, 6, FA, 6, RE, 6, DO, 6, NO, 6, MI, 6, SO, 6,
      FA, 6, NO, 6, DO, 6, MI, 6, NO, 6, FA, 6, SO, 6, NO, 6, 
      MI, 6, RE, 6, NO, 6, DO, 6, MI, 6, SO, 6, FA, 12, END},//でこぼこ道を走るUphill A 
    { MI, 6, NO, 6, FA, 6, SO, 6, RA, 6, SO, 6, FA, 6, MI, 6, 
      RE, 6, DO, 6, NO, 6, RE, 6, MI, 6, FA, 6, SO, 6, FA, 6, 
      NO, 6, MI, 6, SO, 6, NO, 6, RA, 6, SO, 6, FA, 6, MI, 6, 
      RE, 6, NO, 6, DO, 6, RE, 6, MI, 6, FA, 12, END }//でこぼこ道を走るUphill B
} ;

int accMusic = 0 ;
int updownMusic = 0 ;

void setup ()
{
    Serial.begin(115200);
    initializeAcceleSensor () ;
    initializeNote () ;
}

void initializeAcceleSensor ()
{   
    accel.begin (LIS3DH_ADDRESS ) ;
    accel.setClick ( 0, 0 ) ;                      // Disable Interrupt
    accel.setRange ( LIS3DH_RANGE_2_G ) ;          // Full scale +/- 2G
    accel.setDataRate ( LIS3DH_DATARATE_10_HZ );   // Data rate = 10Hz
    delay ( 100 ) ;
}

void initializeNote ()
{
    int     i ;

    for ( i = 0 ; i < usePinCount ; i ++ )
    {
        pinMode ( usePin[i], OUTPUT ) ;
        digitalWrite ( usePin[i], HIGH ) ;
    } 
    delay ( 100 ) ;       
}

void loop ()
{
    timer100 () ;
    timer1000 () ;
    timer30000 () ;
}

void getVibration ( double level )
{
    double  acc ;
    static int count = 0 ;
    static double oldAcc = -1.0 ;

    accel.read () ;
    acc = sqrt ( accel.x_g * accel.x_g + accel.y_g * accel.y_g + accel.z_g * accel.z_g ) ; 
    if ( oldAcc < 0.0 )
    {
        oldAcc = acc ;
    }
    if ( fabs ( acc - oldAcc ) > 0.05 )
    {
        count ++ ;
    }
    else
    {
        count = 0 ;
        if ( accMusic == 1 )
        {
            accMusic = 0 ;
            Serial.print ( "Accel Music Stop\n" ) ;             
        }
    }
    if ( count > 5 && accMusic == 0 )
    {
        accMusic = 1 ; 
        Serial.print ( "Accel Music Start\n" ) ;       
    }
    oldAcc = acc ;    
}

void playMusic ( int id, int nt )
{
    static int t = 0 ;
    static int tcount = 0 ;

    if ( music[id][t*2] == END )
    {
        accMusic = 0 ;
        updownMusic = 0 ;
        t = 0 ;
        return ;
    }

    if ( tcount == 0 )
    {
        digitalWrite ( usePin[music[id][t*2]], LOW ) ;
    }
    if ( tcount == music[id][t*2+1] / 2 )
    {
        digitalWrite ( usePin[music[id][t*2]], HIGH ) ;        
    }      
    tcount ++ ;

    if ( tcount >= music[id][t*2+1] )
    {
        tcount = 0 ;
        t ++ ;
    }
}

void timer100 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;
    static int nt = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
        nt = 0 ;
    }
            
    if ( currentTime - oldTime > 100 )
    {
        if ( accMusic == 1 )
        {
            playMusic ( 0, nt ) ;
        }
        if ( updownMusic == 1 )
        {
            playMusic ( 1, nt ) ;
        }
        oldTime = currentTime ;
        nt ++ ;
    }
}

void timer500 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
    }
            
    if ( currentTime - oldTime > 500 )
    {

        oldTime = currentTime ;
    }
}

void timer1000 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
    }
            
    if ( currentTime - oldTime > 1000 )
    {
        getVibration ( 0.05 ) ;
        oldTime = currentTime ;
    }
}

void timer5000 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
    }
            
    if ( currentTime - oldTime > 5000 )
    {

        oldTime = currentTime ;
    }
}

void timer10000 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
    }
            
    if ( currentTime - oldTime > 10000 )
    {

        oldTime = currentTime ;
    }
}

void timer30000 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
    }
            
    if ( currentTime - oldTime > 30000 )
    {
        updownMusic = 1 ;
        Serial.print ( "Updown Music Start" ) ;
        oldTime = currentTime ;
    }
}

