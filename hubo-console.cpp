#include <stdio.h>
#include <stdlib.h>
#include <readline/readline.h>
#include <readline/history.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../hubo-ach/hubo.h"



// for ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"






#include <iostream>
#include <sstream>
#include <string>
using namespace std;


// ach message type
typedef struct hubo h[1];

// ach channels
ach_channel_t chan_num;


 
static char** my_completion(const char*, int ,int);
char* my_generator(const char*,int);
char * dupstr (char*);
void *xmalloc (int);
void parse(char *s);
int test(char *s , struct hubo *h);
char* getArg(string s, int argNum);
void hubo_update(struct hubo *h);
int name2mot(char*s, struct hubo *h);
double hubo_get(char*s, struct hubo *h);
//char* cmd [] ={ "test","hello", "world", "hell" ,"word", "quit", " " };
char* cmd [] ={ "initialize","zero","fet","ctrl","enczero", "goto","get","test","update", "quit", " " };
 

int main() {
	printf("\n");
	printf(" ***************** hubo-ach **************** \n");
	printf(" Support: Daniel M. Lofaro dan@danlofaro.com \n");
	printf(" ******************************************* \n");
	
        // get initial values for hubo
        // open ach channel
        int r = ach_open(&chan_num, "hubo", NULL);
        assert( ACH_OK == r );

        struct hubo H;
        size_t fs;
        r = ach_get( &chan_num, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
	//      printf("fs = %i, H = %i\n",fs, sizeof(H));
        assert( sizeof(H) == fs );



	char *buf;
	rl_attempted_completion_function = my_completion;

	while((buf = readline("\n hubo-ach: "))!=NULL) {
	//enable auto-complete
	rl_bind_key('\t',rl_complete);
 
	printf("cmd [%s]\n",buf);

	/* get update after every command */
	hubo_update(&H);


	char* buf0 = getArg(buf, 0);
	//printf(buf0);	

	if (strcmp(buf0,"update")==0) {
		hubo_update(&H);
		printf("--->Hubo Information Updated\n");
	}

	if (strcmp(buf0,"get")==0) {
		double jRef = hubo_get(buf,&H);
		char* tmp = getArg(buf,1);
		printf(">> %s = %f rad \n",tmp,jRef);
	}
	if (strcmp(buf0,"test")==0)
		test(buf, &H);
	/* Quit */
	if (strcmp(buf0,"quit")==0)
		break;
	if (buf[0]!=0)
	add_history(buf);
	}
 
	free(buf);
 	return 0;
}

double hubo_get(char*s, struct hubo *h) {

	/* get joint number */
	int jointNo = name2mot(getArg(s,1),h);

	return h->joint[jointNo].ref;
}


void hubo_update(struct hubo *h) {
       	size_t fs;
       	int r = ach_get( &chan_num, h, sizeof(*h), &fs, NULL, ACH_O_LAST );
//	printf("r = %i\n",r);
//	printf("sizefo(*h) = %i, fs = %i\n",sizeof(*h), fs);
	//if((r != ACH_STALE_FRAMES) & (r != ACH_MISSED_FRAME)) {
	if((r == ACH_OK) | (r == ACH_MISSED_FRAME)) {
	       	assert( sizeof(*h) == fs );
	}

	// look into posix message que
	// posix rt signal can give signal numb er and an interger
}


char* getArg(string s, int argNum) {
//	printf("\n dan test\n");

	istringstream iss(s);
	
	int i = 0;
//	char* theOut[];
	do
    	{
        	string sub;
        	iss >> sub;
 //       	cout << "Substring: " << sub << endl;
		if( i == argNum ) {
			return (char*)sub.c_str(); }
		i++;
    	} while (iss);

    	return NULL;
}

int test(char* s, struct hubo *h) {
	printf("\n dan test\n");

	char* mot = getArg(s,1);

	int tmp = name2mot(mot, h);
	printf("mot = %i \n",tmp);
	return 0;
}

int name2mot(char* name, struct hubo *h) {
	/* Returns the number of the requested joint */	
	int i = 0;
	int iout = -1;
	for( i = 0; i < numOfJoints; i++ ) {
			char *mot = h->joint[i].name;
			if (strcmp(name, mot) == 0) {
//				printf("i = %i\n", i);
				iout = i;}
	}
	return iout;
}


 
static char** my_completion( const char * text , int start,  int end) {
	char **matches;
 
	matches = (char **)NULL;
 
	if (start == 0)
		matches = rl_completion_matches ((char*)text, &my_generator);
	else
		rl_bind_key('\t',rl_abort);
 
	return (matches);
}
 
char* my_generator(const char* text, int state) {
	static int list_index, len;
	char *name;
 
	if (!state) {
		list_index = 0;
		len = strlen (text);
	}
 
	while (name = cmd[list_index]) {
		list_index++;
 
	if (strncmp (name, text, len) == 0)
		return (dupstr(name));
	}
 
	/* If no names matched, then return NULL. */
	return ((char *)NULL);
}
 
char * dupstr (char* s) {
	char *r;
 
	r = (char*) xmalloc ((strlen (s) + 1));
	strcpy (r, s);
	return (r);
}
 
void * xmalloc (int size) {
	void *buf;
 
	buf = malloc (size);
	if (!buf) {
		fprintf (stderr, "Error: Out of memory. Exiting.'n");
		exit (1);
	}
	return buf;
}
