/*
 atexit workaround from 
 http://www.geocrawler.com/archives/3/360/2001/8/0/6491254/
*/

extern int atexit (void (*__func) (void));

void atexit0 ( void ) {
}

int dummy0 ( void ) {
        atexit( atexit0 );
        return 0;
}
