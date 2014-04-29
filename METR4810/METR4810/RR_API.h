#ifndef _RR_API
#define _RR_API 1

#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// default read and write socket timeout
#define DEFAULT_TIMEOUT 60000

// the port number to listen on ... needs to match that used in RR interface
#define SERVER_PORTNUM 6060

// maximum size of a message that can be read from RoboRealm
#define MAX_BUFFER_SIZE 4096

class RR_API
{
	// windows socket specific data structures
	WSADATA winsock_data;
	WORD version_required; /* Version 1.1 */
  int handle;

	// holds the previously read data size
	int lastDataTop;
	// holds the previously read data buffer
	int lastDataSize;
	// contains the read/write socket timeouts
	int timeout;

	char errorMsg[64];
	int readMessage(int hSocket, unsigned char *buffer, int len);
	int readImageData(int hSocket, unsigned char *buffer, int len);
	void unescape(char *txt);
	void escape(char *txt, char *dest, int max);

public:

	// indicates that the application is connected to RoboRealm Server
	bool connected;
	// general buffer for data manipulation and socket reading
	char buffer[MAX_BUFFER_SIZE];

	~RR_API();
	RR_API();
	bool connect(char *hostname, int port=SERVER_PORTNUM);
	bool getDimension(int *width, int *height);
	bool getImage(char *name, unsigned char *pixels, int *width, int *height, unsigned int len, char *mode);
	bool getImage(unsigned char *pixels, int *width, int *height, unsigned int len);
	void disconnect();
	bool getVariable(char *name, char *buffer, int max);
	bool setVariable(char *name, char *value);
	bool deleteVariable(char *name);
	bool setImage(unsigned char *image, int width, int height, bool wait=false, char *mode="RGB");
	bool setImage(char *name, unsigned char *image, int width, int height, bool wait=false, char *mode="RGB");
	bool setCompressedImage(unsigned char *image, int size, bool wait=false);
	bool setCompressedImage(char *name, unsigned char *image, int size, bool wait=false);
	bool execute(char *source);
	bool loadProgram(char *filename);
	bool saveProgram(char *filename);
	bool loadImage(char *name, char *filename);
	bool saveImage(char *source, char *filename);
	int read(int hSocket, unsigned char *buffer, int len);
	bool run(char *mode);
	bool waitVariable(char *name, char *value, int timeout);
	bool waitImage(int timeout=5000);
	bool close();
	bool open(char *filename, int port=6060);
	unsigned char *readLine(FILE *fp, unsigned char *buffer);
	int savePPM(char *filename, unsigned char *buffer, int width, int height);
	int loadPPM(char *filename, unsigned char *buffer, int *width, int *height, int max);
	int getVariables(char *names, char *results[], int len, int rows);
	bool setVariables(char *names[], char *values[], int num);
	void skipData(int hSocket, int len);
	bool getParameter(char *module, int count, char *name, char *result, int max);
	bool setParameter(char *module, int count, char *name, char *value);
	bool getProgram(char *xml, int len);
	HBITMAP getBitmap(char *name, unsigned char *pixels, int *width, int *height, unsigned int max);
	bool minimizeWindow();
	bool maximizeWindow();
	bool moveWindow(int x, int y);
	bool resizeWindow(int width, int height);
	bool positionWindow(int x, int y, int width, int height);
	bool setCamera(char *mode);
	bool getCamera(char *mode, int max);
	bool setCameraFormat(int width, int height, int frameRate=0, char compression[4]="");
	bool getCameraFormat(int *width, int *height, int *frameRate=NULL, char compression[4]=NULL);
	int parseProperyTagset(char *buffer, int i, char *name, int *value, int *min, int *max, int *automatic);
	bool getCameraProperties(char **names, int *values, int *min, int *max, int *automatic, int num);
	bool setCameraProperties(char **names, int *values, int *automatic, int num);
	bool showWindow(bool show);
	bool getVersion(char *version);
	bool deleteModule(char *name, int count=0);
	bool toggleModule(char *name, int count=0);
};

#endif
