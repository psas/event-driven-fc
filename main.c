#include <stdio.h>
#include <string.h>

#include "control.h"

int main ( int argc, char *argv[] ) {

  bool noisy = false;

  while ( ++argv, --argc ) {
    if ( strcmp( argv[0], "-v" ) == 0 )
      noisy = true;
    else {
      fprintf( stderr, "Unknown argument %s!\n", argv[0] );
      return 1;
    };
  };

  run_flight_control( noisy );

  return 0;

};
