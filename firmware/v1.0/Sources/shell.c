#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdarg.h"

#include "main.h"
#include "ringbuffer.h"
#include "shell.h"
#include "tcp_server.h"


#if 1



#ifdef   DEBUG_LOG
   #define  dprintf(...)           DEBUG_LOG(""__VA_ARGS__);
#else
   #define  dprintf(...)      {  }
#endif
#if 1
#define cmd_printf(...)      { cmd_print(terminal,__VA_ARGS__); }
#else
#warning cmd_printf
#define cmd_printf(...)	{  }
#endif
typedef int (  DYN_SHELL_CALLBACK ) (SHELL_TERMINAL_ST *terminal, int argc, char ** argv );

static SHELL_TERMINAL_ST terminal[] = {
  {
    .getchar = Uart2Getchar,
    .putchar = Uart2Putchar,
    .buffer_idx = 0,
  },

  {
      .getchar = TcpGetchar,
      .putchar = TcpPutchar,
      .buffer_idx = 0,
    }

};


struct DYN_SHELL {
    DYN_SHELL_CALLBACK *dynshell_function;
    char functionname[10];
    char note[20];
};

static int     SHELL_pharse       ( char * BUFFER, char ** argv, int max_argc );
static int     SHELL_runcmd       ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv);

static int     cmd_help           ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv);
static int     cmd_ver            ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv);
static int     cmd_c              ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv);
static int     cmd_dump           ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv);
static int     cmd_agc            ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv);
static int     cmd_snr            ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv);
static int     cmd_array          ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv);
static int     cmd_search         ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv);
#if 1
static const struct __attribute__((packed))  DYN_SHELL shell_cmd_table [ MAX_SHELL_ENTRYS ]  = {

    { cmd_help,           "-h",           "help"           },
    { cmd_ver,            "-v",           "version"        },
	{ cmd_c,              "c",            "cap [ch] [val]" },
	{ cmd_agc,            "a",            "agc [ch] [val]" },
	{ cmd_snr,            "m",            "magnitudo"      },
	{ cmd_array,          "d",            "get 1 sec data" },
	{ cmd_search,         "s",            "search freq"    },
	{ cmd_dump,           "?",            "dump"           },

};
#else
#warning shell_size

static const struct __attribute__((packed)) DYN_SHELL shell_cmd_table [ 0 ]  = {

   // { cmd_help,           "-h",           "help"           },
};
#endif


/**
 * @brief
 * @param[in] BUFFER input array pointer
 * @param[in] argv arg array pointer
 * @param[in] max_argc rag max count
 * @return Error code
 */
int SHELL_pharse( char * BUFFER, char ** argv, int max_argc ) {

	int i     = 0;
	int _argc = 0;
	char toggle = 0;
	int stringlen = strlen( BUFFER );

	argv[ _argc ] = &BUFFER[ i ];


	for(i = 0 ; i < stringlen && _argc < max_argc ; i++ )
	{
		switch( BUFFER[ i ] )
		{
        case ';':		i = stringlen;
						break;
		case '\0':		i = stringlen;
						break;
        case 0x0a:		BUFFER[ i ] = '\0';
						i = stringlen;
						break;
		case 0x0d:		BUFFER[ i ] = '\0';
						i = stringlen;
						break;
		case '\"':		if ( toggle == 0 ) {
							toggle = 1;
							BUFFER[ i ] = '\0';
							argv[ _argc ] = &BUFFER[ i + 1];
						} else {
							toggle = 0;
							BUFFER[ i ] = '\0';
						}
						break;
		case ' ':		if ( toggle == 0 )
						{
							BUFFER[ i ] = '\0';
							_argc++;
							argv[ _argc ] = &BUFFER[ i + 1 ];
						}
						break;
		}
	}
	return( _argc + 1 );
}
/**
 * @brief
 * @param[in] argc arg count
 * @param[in] argv arg array pointer
 * @return Error code
 */
int SHELL_runcmd(SHELL_TERMINAL_ST *terminal, int argc, char ** argv ) {
    int returncode = 1;


    unsigned int cnt = ITEMNUM(shell_cmd_table);

   for( int i = 0 ; i < cnt ; i++ ) {
		   char name[30];
		   DYN_SHELL_CALLBACK * pfunc = NULL;
		   memcpy(name, &shell_cmd_table[i].functionname, sizeof (name));


			if ( !strcmp( argv[0] , name ) ) {
                memcpy(&pfunc, &shell_cmd_table[i].dynshell_function, sizeof (pfunc));
                if (pfunc != NULL){
                    returncode =(*pfunc)(terminal, argc, argv );
                } else {
                    returncode = 0;
                }
				break;
			}
    }

	return( returncode );
}

void cmd_print (SHELL_TERMINAL_ST *terminal,char * format, ...) {
#if 1
  char buffer[256];
  memset(buffer,0,sizeof(buffer));

  va_list args;
  va_start (args, format);
  vsnprintf (buffer, sizeof(buffer)-1, format, args);
  va_end (args);

  char *s = buffer;

  while(*s){
		(*terminal->putchar)(*s++);
	}
# else
#warning cmd_print
#endif
}
/**
 * @brief
 * @param[in] argc Arg count
 * @param[in] argv Arg array
 * @return Error code
 */
int cmd_help    (SHELL_TERMINAL_ST *terminal, int argc, char ** argv){
    cmd_printf("usage commands: \r\n");
    for(int i = 0; i< MAX_SHELL_ENTRYS ; i++){
        if (shell_cmd_table[i].dynshell_function != NULL ){
            cmd_printf("  %.30s : %.30s\r\n",shell_cmd_table[i].functionname,shell_cmd_table[i].note);
        }
    }
    return 0;
}

/**
 * @brief
 * @param[in] argc Arg count
 * @param[in] argv Arg array
 * @return Error code
 */
int cmd_ver    (SHELL_TERMINAL_ST *terminal, int argc, char ** argv){
    /*
    #define STR_INDIR(x) #x
    #define STR(x) STR_INDIR(x)

    cmd_printf("Build      : %s \r\n",BUILD_DATE);
    cmd_printf("Build date : %u.%02u.%02u \r\n",BUILD_YEAR,BUILD_MONTH,BUILD_DAY);
    cmd_printf("Version GIT: %s \r\n",VERSION );
    cmd_printf("Version    : %u.%u.%u.%u %s %s \r\n",VERSION_MAJOR,VERSION_MINOR,VERSION_BUILDNO,VERSION_EXTEND, VERSION_DATE,VERSION_TIME);
    cmd_printf("VERSION_FULL:%s \r\n", STR(VERSION_FULL));
  */

    return 0;
}

int cmd_search    (SHELL_TERMINAL_ST *terminal, int argc, char ** argv){

	search = !search;

    return 0;
}


static int     cmd_c              ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv){

	int ch    = -1;
    int value = -1;
	if (argc >= 3){
	        ch    = atoi(&argv[1][0]);
	        value = atoi(&argv[2][0]);


	        switch(ch){
				case 0: {
					if (value) {
						RFC1_high();
					} else {
						RFC1_low();
					}
				} break;
				case 1: {
					if (value) {
						RFC2_high();
					} else {
						RFC2_low();
					}

				} break;
				case 2: {
					if (value) {
						RFC3_high();
					} else {
						RFC3_low();
					}

				} break;
				case 3: {
					if (value) {
						RFC4_high();
					} else {
						RFC4_low();
					}
				} break;
				default : {

				}
	        	}


	        cmd_printf("CAP[%d] %s  (0x%X)\r\n",ch,value==0?"low":"high",cap);
	 }


	return 0;

}
static int     cmd_dump           ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv){

	cmd_printf("CAP (0x%X)\r\n",cap);
	cmd_printf("AGC (0x%X)\r\n",agc);
	cmd_printf("SNR : %d , min %d, max %d\r\n", (int)snr, (int)min, (int) max );
	datetime_t dt;
	uint64_t now  = get_unix_time_us();
	unix_us_to_datetime(now, &dt);


	cmd_printf("TIME: 20");
	cmd_printf("%d",dt.yr-2000);
	cmd_printf("-");
	cmd_printf("%s",dt.mon<10?"0":""); cmd_printf("%d",dt.mon);
	cmd_printf("-");
	cmd_printf("%s",dt.day<10?"0":""); cmd_printf("%d",dt.day);
	cmd_printf(" ");
	cmd_printf("%s",dt.hr<10?"0":""); cmd_printf("%d",dt.hr);
	cmd_printf(":");
	cmd_printf("%s",dt.min<10?"0":""); cmd_printf("%d",dt.min);
	cmd_printf(":");
	cmd_printf("%s",dt.sec<10?"0":""); cmd_printf("%d\r\n",dt.sec);



	return 0;
}

static int     cmd_array           ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv){
    if (send_array){
		cmd_printf("Wait time...\r\n");
		send_array = 1;
    } else {
    	send_array = 0;
    	cmd_printf("Stop waiting...\r\n");
    }
    return 0;

}

static int     cmd_snr           ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv){

	extern int max_snr1;
	extern int max_snr2;
	extern int min_snr1;
	extern int min_snr2;

	cmd_printf("SNR : %d , adc ( %d,  %d) snr ( %d,  %d) / ( %d,  %d)\r\n", (int)snr, (int)min, (int) max, min_snr1,
			max_snr1,min_snr2 ,max_snr2);

	max_snr1 = INT_MIN;
	max_snr2 = INT_MIN;
	min_snr1 = INT_MAX;
	min_snr2 = INT_MAX;
	return 0;
}



extern void cmd_agc_power(int range);

static int     cmd_agc            ( SHELL_TERMINAL_ST *terminal, int argc, char ** argv){
	int ch    = -1;
	    int value = -1;

	    if (argc == 2){
	        value = atoi(&argv[1][0]);
	        cmd_agc_power(value);
            cmd_printf("AGC[%d] %s (0x%X)\r\n",ch,value==0?"low":"high", agc);
            agc_disable = 1;
	    }

		if (argc >= 3){
                agc_disable = 1;
		        ch    = atoi(&argv[1][0]);
		        value = atoi(&argv[2][0]);
		        switch(ch){
					case 0: {
						if (value) {
							AGC1_high();
						} else {
							AGC1_low();
						}
					} break;
					case 1: {
						if (value) {
							AGC2_high();
						} else {
							AGC2_low();
						}

					} break;
					case 2: {
						if (value) {
							AGC3_high();
						} else {
							AGC3_low();
						}

					} break;
					case 3: {
						if (value) {
							AGC4_high();
						} else {
							AGC4_low();
						}
					} break;
					default : {

					}
				}

		        cmd_printf("AGC[%d] %s (0x%X)\r\n",ch,value==0?"low":"high", agc);
		 }

		return 0;

}


int Shell (SHELL_TERMINAL_ST *terminal,char * str){
  int retval = 0;
  char *argv[MAX_ARGC + 1];
  int  argc;

  argc    = SHELL_pharse( str,   argv, MAX_ARGC );
  retval  = SHELL_runcmd(terminal, argc, argv );
  return retval;

}


void ShellTask(void){

//    int ch = Uart2Getchar();
//	if (ch != -1) {
//		Uart2Putchar(ch);
//	}
//	return;



    for(int i = 0; i < ITEMNUM(terminal);i++) {

      int16_t ch = (*terminal[i].getchar)();

      if (ch != -1) {
        while(1) {

            if (terminal[i].buffer_idx < SHELL_BUFFER_LEN){
              terminal[i].buffer[ terminal[i].buffer_idx++ ] = ch;
            } else {
              terminal[i].buffer_idx = 0;

               memset(&terminal[i].buffer[0],0,sizeof(terminal[i].buffer));
            }

            if ((ch == '\n') || (ch == '\r')) {

             Shell(&terminal[i], &terminal[i].buffer[0]);

             terminal[i].buffer_idx = 0;
             memset(&terminal[i].buffer[0],0,sizeof(terminal[i].buffer));
            }

            ch = (*terminal[i].getchar)();
            if (ch == -1) break;

        }

      }


    }
}
#endif

