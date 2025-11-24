#include <Adafruit_LIS3DH.h>

#define LIS3DH_ADDRESS 0x19
Adafruit_LIS3DH accel = Adafruit_LIS3DH ();

double GPSHeight = 0.0 ;

int usePinCount = 8 ;
int usePin[] = { D0, D4, D6, D7, D8, D10, D11, D12 } ; 
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

int musicMode = 0 ;
int music[][256] = {
    { DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, FA, 6, MI, 6, RE, 6, 
      DO, 6, RE, 6, MI, 6, FA, 6, FA, 12, 
      DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, FA, 6, MI, 6, RE, 6, 
      DO, 6, RE, 6, MI, 6, FA, 6, FA, 18, NO, 30, END
    },
    { RE, 6, MI, 6, FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, 
      FA, 6, MI, 6, RE, 6, MI, 6, FA, 12, 
      MI, 6, FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, 
      MI, 6, FA, 6, MI, 6, RE, 6, FA, 18, NO, 30, END
    },
    { FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, 
      FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 12, 
      MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, RE, 6, 
      DO, 6, RE, 6, MI, 6, FA, 6, MI, 12, NO, 30, END
    },  
    { MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, RE, 6,
      DO, 6, RE, 6, MI, 6, FA, 6, MI, 12,
      FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, 
      RE, 6, DO, 6, RE, 6, MI, 6, FA, 18, NO, 30, END
    },
    { DO, 6, NO, 6, RE, 6, MI, 6, FA, 6, MI, 6, RE, 6,
      DO, 6, NO, 6, MI, 6, NO, 6, FA, 6, NO, 6, MI, 6, NO, 6, 
      DO, 6, RE, 6, NO, 6, MI, 6, FA, 6, NO, 6, RE, 6, NO, 6, 
      DO, 6, MI, 6, FA, 12, NO, 30, END
    },
    { MI, 6, NO, 6, FA, 6, RE, 6, DO, 6, NO, 6, RE, 6, MI, 6,
      FA, 6, NO, 6, MI, 6, RE, 6, DO, 6, NO, 6, RE, 6, NO, 6, 
      MI, 6, NO, 6, FA, 6, RE, 6, DO, 6, NO, 6, RE, 6, MI, 6,
      FA, 6, NO, 6, RE, 6, NO, 6, DO, 6, MI, 6, FA, 12, NO, 30, END
    },
    { NO, 100, END }
} ;

int orderCount = 50 ;
int order[] = {
     0, 0, 0, 0, 6, 0, 0, 0, 0, 6, 
     1, 1, 1, 1, 6, 1, 1, 1, 1, 6, 
     2, 2, 2, 2, 6, 2, 2, 2, 2, 6,
     3, 3, 3, 3, 6, 3, 3, 3, 3, 6,
     4, 4, 4, 4, 6, 5, 5, 5, 5, 6 } ;
double speed[] = {
    1.0, 1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
} ;

int accMusic = 0 ;
int upMusic = 0 ;
int downMusic = 0 ;
int selectMusic = 0 ;

//music functions
void initializeNote () ;
void initializeAcceleSensor () ;
void getHeight ( double height ) ;
void getVibration ( double level ) ;
int playMusic ( int id, double ts ) ;
void timer100 () ;
void timer500 () ;
void timer1000 () ;
void timer5000 () ;
void timer10000 () ;
void timer30000 () ;

void setup ()
{
    Serial.begin ( 9600 );
    //加速度センサー初期化
    initializeAcceleSensor () ;
    //使用するすべてのピンを出力モードに設定
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
        pinMode ( usePin[i], OUTPUT ) ; //使用するピンを出力モードに設定
        digitalWrite ( usePin[i], HIGH ) ;  //ピンをHIGHに設定（おはようモード）
    } 
    delay ( 100 ) ;       
}

void loop ()
{
    timer100 () ;
    timer1000 () ;
    timer30000 () ;
}

void getHeight ( double height )
{
    static double   oldHeight = -1.0 ;

    if ( oldHeight < 0.0 )
    {
        oldHeight = GPSHeight ;
    }
    if ( GPSHeight - oldHeight > height )
    {
        upMusic = 1 ;
        selectMusic = rand () % 2 ;
        Serial.print ( "Up Music Start" ) ; 
    }
    else if ( GPSHeight - oldHeight < -height )
    {
        downMusic = 1 ;
        selectMusic = rand () % 2 ; 
        Serial.print ( "Up Music Start" ) ; 
    }
    else
    {
        upMusic = 0 ;
        downMusic = 0 ;
    }
    oldHeight = GPSHeight ;
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
    if ( fabs ( acc - oldAcc ) > level )
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
        selectMusic = rand () % 2 ;       
    }
    oldAcc = acc ;    
}

int playMusic ( int id, double ts )
{
    static int t = 0 ;
    static int tcount = 0 ;

    if ( music[id][t*2] == END )
    {
        t = 0 ;
        return 1 ;
    }

    if ( tcount == 0 )
    {
        if ( music[id][t*2] != NO )
        {
            digitalWrite ( usePin[music[id][t*2]], LOW ) ;
            delay ( ( int )( music[id][t*2+1] * 100.0 * ( 0.005 * ts ) ) ) ;
            digitalWrite ( usePin[music[id][t*2]], HIGH ) ;
        }
    }   
    tcount ++ ;

    if ( tcount >= ( int ) ( music[id][t*2+1] * ts ) )
    {
        tcount = 0 ;
        t ++ ;
    }
    return 0 ;
}


/*
 *  timer func 0.1s
 */
void timer100 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;
    static int id = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
    }
            
    if ( currentTime - oldTime > 100 )
    {
        if ( playMusic ( order[id], speed[id] ) == 1 )
        {
            id ++ ;
            id = id % orderCount ;
        }
        oldTime = currentTime ;
    }
}

/*
 *  timer func 0.5s
 */
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

/*
 *  timer func 1s
 */
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

/*
 *  timer func 5s
 */
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

/*
 *  timer func 10s
 */
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
        getHeight ( 0.5 ) ;
        Serial.print ( "Updown Music Start" ) ;
        oldTime = currentTime ;
    }
}

/*
 *  timer func 30s
 */
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
        oldTime = currentTime ;
    }
}