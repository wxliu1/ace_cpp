
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <chrono>
#include <thread>

int getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                 // disable buffering      
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	int c = getchar();  // read character (non-blocking)
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	return c;
}

int main(int argc, char* argv[])
{
    //
    for(;;)
    {
        std::cout << "c=" << getch() << std::endl;
        std::chrono::milliseconds dura(500);
        std::this_thread::sleep_for(dura);
    }

    return 0;
}