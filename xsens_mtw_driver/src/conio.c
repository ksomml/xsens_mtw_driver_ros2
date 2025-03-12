
//  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//

#include <sys/select.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include "../include/xsens/conio.h"

/*! getch hack

	Don't use in production code!
*/
int _getch(void)
{
	struct termios oldt, newt;
	int c;

	tcgetattr(0, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(0, TCSANOW, &newt);
	c = getchar();
	tcsetattr(0, TCSANOW, &oldt);
	return c;
}

/*! kbhit hack

	Don't use in production code!
*/
int _kbhit()
{
	struct termios oldt, newt;

	struct timeval tv;
	fd_set rfd;
	tv.tv_sec = 0;
	tv.tv_usec = 0;

	FD_ZERO(&rfd);
	FD_SET(0, &rfd);

	tcgetattr(0, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(0, TCSANOW, &newt);

	if (select(1, &rfd, 0, 0, &tv) == -1)
		return 0;

	tcsetattr(0, TCSANOW, &oldt);

	return (FD_ISSET(0, &rfd) != 0);
}
