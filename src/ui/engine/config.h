#ifndef CONFIG_H
#define CONFIG_H

enum {
	D_NOTHING			= 0,
	D_CONSTRUCTORS		= 1,
	D_ALL_CONSTRUCTORS	= 2,
	D_DESTRUCTORS		= 4,
	D_PARAMS			= 8,
	D_ALL_PARAMS		= 16,
	D_SHADERS			= 32,
	D_BUFFER			= 64,
	D_WARNINGS		= 128,
	D_ERRORS			= 256,
	D_EVERYTHING		= 511
};

/* For more info, see ConfigFile wiki page. */
struct sGlobalConfig {
	
	/* Configures the output verbosity. */
	static unsigned	DEBUGGING;
  static bool CREATE_MIPMAPS;
};

#endif