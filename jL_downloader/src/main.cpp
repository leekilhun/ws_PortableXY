/*
 * main.cpp
 *
 *  Created on: 2022. 9. 13.
 *      Author: gns2l
 *
 *
 * jL_downloader PC실행 파일
 * 강제종료  Ctrl +C
 *
 */

#include "main.h"

void exitISR(int sig)
{
  apExit();
}


int main(int argc, char *argv[])
{
  signal(SIGINT, exitISR);
  setbuf(stdout, NULL);

  hwInit();
  apInit();

  apMain(argc, argv);

  return 0;
}

