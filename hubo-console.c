#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>

#include <string.h>
#include <inttypes.h>

// For ach
#include "ach.h"

// For hubo
#include "../hubo-ach/hubo.h"


/* for autocomplete */
#include <sys/types.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/errno.h>

#include <readline/readline.h>
#include <readline/history.h>

#define NSEC_PER_SEC    1000000000

// ach message type
//typedef struct hubo HUBO[1];

// ach channels
ach_channel_t chan_num;

extern char *getwd ();
extern char *xmalloc ();

/* The names of functions that actually do the manipulation. */
int com_cd ();

/* A structure which contains information on the commands this program
   can understand. */

typedef struct {
  char *name;			/* User printable name of the function. */
  Function *func;		/* Function to call to do the job. */
  char *doc;			/* Documentation for this function.  */
} COMMAND;


struct console C;

COMMAND commands[] = {
  { "cd", com_cd, "Change to directory DIR" },
  { (char *)NULL, (Function *)NULL, (char *)NULL }
};

int com_cd(char* aa) {
	printf("dan\n");
	return 1;
}



/* Look up NAME as the name of a command, and return a pointer to that
   command.  Return a NULL pointer if NAME isn't a command name. */
COMMAND *
find_command (name)
     char *name;
{
  register int i;

  for (i = 0; commands[i].name; i++)
    if (strcmp (name, commands[i].name) == 0)
      return (&commands[i]);

  return ((COMMAND *)NULL);
}



/* Execute a command line. */
int
execute_line (line)
     char *line;
{
  register int i;
  COMMAND *command;
  char *word;

  /* Isolate the command word. */
  i = 0;
  while (line[i] && whitespace (line[i]))
    i++;
  word = line + i;

  while (line[i] && !whitespace (line[i]))
    i++;

  if (line[i])
    line[i++] = '\0';

  command = find_command (word);

  if (!command)
    {
      fprintf (stderr, "%s: No such command for FileMan.\n", word);
      return (-1);
    }

  /* Get argument to command, if any. */
  while (whitespace (line[i]))
    i++;

  word = line + i;

  /* Call the function. */
  return ((*(command->func)) (word));
}





/* Strip whitespace from the start and end of STRING.  Return a pointer
   into STRING. */
char *
stripwhite (string)
     char *string;
{
  register char *s, *t;

  for (s = string; whitespace (*s); s++)
    ;
    
  if (*s == 0)
    return (s);

  t = s + strlen (s) - 1;
  while (t > s && whitespace (*t))
    t--;
  *++t = '\0';

  return s;
}

void consoleLoop(struct console *C) {
/* gui for controling basic features of the hubo  */
        printf("hubo-ach - interface 2012-08-18\n");
//	ach_put(&chan_num, &H, sizeof(H));


	/* Forward declarations. */
	char *stripwhite ();
	COMMAND *find_command ();

	/* The name of this program, as taken from argv[0]. */
	char *progname;

	/* When non-zero, this global means the user is done using this program. */
	int done;


	char *line, *s;

	int fconsole = 1;
	while(fconsole) {
	 line = readline ("hubo-ach: ");

      	if (!line)
       	 	break;

      /* Remove leading and trailing whitespace from the line.
         Then, if there is anything left, add it to the history list
         and execute it. */
      	s = stripwhite (line);

      	if (*s){
          add_history (s);
          execute_line (s);
        }

      	free (line);

	}
}
// read line library


int main(int argc, char **argv){
	(void) argc; (void)argv;

	//struct hubo C;
	size_t fs;
	// open ach channel
        int r = ach_open(&chan_num, "hubo-ach-console", NULL);
        assert( ACH_OK == r );

	r = ach_get( &chan_num, &C, sizeof(C), &fs, NULL, ACH_O_LAST );
	assert( sizeof(C) == fs );
	
	consoleLoop(&C);
	printf("*** Exiting Hubo-Ach-Console ***\n");
	return 0;

}


