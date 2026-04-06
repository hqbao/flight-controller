#include "dblink.h"

/* Internal setup functions — defined in individual .c files */
extern void db_reader_setup(void);
extern void db_sender_setup(void);

void dblink_setup(void) {
	db_reader_setup();
	db_sender_setup();
}
