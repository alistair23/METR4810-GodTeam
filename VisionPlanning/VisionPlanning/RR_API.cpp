#define _CRT_SECURE_NO_WARNINGS 1
#define _CRT_SECURE_NO_DEPRECATE 1
#pragma warning(disable: 4996)
#include "RR_API.h"

#pragma comment(lib, "ws2_32.lib")

RR_API::RR_API()
{
	timeout=DEFAULT_TIMEOUT;
	lastDataTop=0;
	lastDataSize=0;
	connected=false;
}

RR_API::~RR_API()
{
	if (connected)
    closesocket(handle);
}

/******************************************************************************/
/* Text string manipulation routines */
/******************************************************************************/

/*
Escapes strings to be included in XML message. This can be accomplished by a
sequence of replace statements.
	& -> &amp;
	" -> &quote;
	< -> &lt;
	> -> &gt;
*/
void RR_API::escape(char *txt, char *dest, int max)
{
	int i, j;

	for (j=i=0;txt[i]&&(j<max);i++)
	{
		if (txt[i]=='&')
		{
			if (j+5<max)
			{
				strcpy(&dest[j], "&amp;");
				j+=5;
			}
		}
		else
		if (txt[i]=='"')
		{
			if (j+6<max)
			{
				strcpy(&dest[j], "&quot;");
				j+=6;
			}
		}
		else
		if (txt[i]=='<')
		{
			if (j+4<max)
			{
				strcpy(&dest[j], "&lt;");
				j+=4;
			}
		}
		else
		if (txt[i]=='>')
		{
			if (j+4<max)
			{
				strcpy(&dest[j], "&gt;");
				j+=4;
			}
		}
		else
		{
			dest[j++]=txt[i];
		}
	}

	dest[j]=0;
}

/*
Unescapes strings that have been included in an XML message. This can be
accomplished by a sequence of replace statements.
	&amp; -> &
	&quote; -> "
	&lt; -> <
	&gt; -> >
*/
void RR_API::unescape(char *txt)
{
	int i, j;

	for (j=i=0;txt[i];i++)
	{
		if (txt[i]=='&')
		{
			if (strnicmp(&txt[i], "&amp;", 5)==0)
			{
				txt[j++]='&';
				i+=4;
			}
			else
			if (strnicmp(&txt[i], "&quote;", 7)==0)
			{
				txt[j++]='"';
				i+=6;
			}
			else
			if (strnicmp(&txt[i], "&lt;", 4)==0)
			{
				txt[j++]='<';
				i+=3;
			}
			else
			if (strnicmp(&txt[i], "&gt;", 4)==0)
			{
				txt[j++]='>';
				i+=3;
			}
		}
		else
		{
			txt[j++]=txt[i];
		}
	}

	txt[j]=0;
}

#if _MSC_VER <= 1310
/*
	Simple sscanf_s (save sscanf) routine for pre vc2005 compilers. Note this only implements
	what is needed for tag parsing.
*/
int sscanf_s(char *buffer, char *format, ...)
{
	int i=0, k=0;

	va_list args;
	va_start(args, format);

	int assigned=0;

	while (buffer[i])
	{
		// look for special format command
		if (format[k]=='%')
		{
			if (memcmp(&format[k], "%*[^>]", 6)==0)
			{
				k+=6;
				while (buffer[i]&&(buffer[i]!='>')) i++;
			}
			else
			if (memcmp(&format[k], "%[^<]", 5)==0)
			{
				k+=5;
        char *s = va_arg(args, char *);
        int l = va_arg(args, int);
				int j=0;
				while (buffer[i]&&(buffer[i]!='<'))
				{
					if (j<l) s[j++] = buffer[i];
					i++;
				}
				if (j<l) { s[j]=0; assigned++; }
			}
			else
				return assigned;
		}
		else
		{
			//otherwise just match characters directly
			if (buffer[i]==format[k])
			{
				i++;
				k++;
			}
			else
			{
				va_end(args);
				// just return on a missmatch
				return assigned;
			}
		}
	}

	va_end(args);

	return assigned;
}
#endif


/******************************************************************************/
/* Socket Routines */
/******************************************************************************/

/* Initiates a socket connection to the RoboRealm server */
bool RR_API::connect(char *hostname, int port)
{
	connected=false;

  version_required = 0x0101; /* Version 1.1 */
  WSAStartup (version_required, &winsock_data);

  int sockaddr_in_length = sizeof(struct sockaddr_in);
  int fromlen = sizeof(struct sockaddr_in);

  if ((handle = socket(AF_INET, SOCK_STREAM, 0))<0)
  {
	  strcpy(errorMsg, "Could not create socket!");
		return false;
  }

  int enable=1;

  if ((setsockopt(handle, SOL_SOCKET, SO_REUSEADDR, (char *)&enable, sizeof(enable)))<0)
  {
		strcpy(errorMsg, "Could not set socket option SO_REUSEADDR!");
    return false;
  }

  if ((setsockopt(handle, SOL_SOCKET, SO_KEEPALIVE, (char *)&enable, sizeof(enable)))<0)
  {
	  strcpy(errorMsg, "Could not set socket option SO_KEEPALIVE!");
    return false;
  }

  if ((setsockopt(handle, IPPROTO_TCP, TCP_NODELAY, (char *)&enable, sizeof(enable)))<0)
  {
	  strcpy(errorMsg, "Could not set socket option TCP_NODELAY!");
    return false;
  }

  struct linger ling;
  ling.l_onoff=1;
  ling.l_linger=10;
  if ((setsockopt(handle, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(linger)))<0)
  {
	  strcpy(errorMsg, "Could not set socket option SO_LINGER!");
    return false;
  }

  sockaddr_in sockaddr;
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_port = htons(port);

  struct hostent *remote_host;        /* internet numbers, names */
  if ((remote_host=gethostbyname(hostname))==(struct hostent *)NULL)
  {
	  _snprintf(errorMsg, 64, "Could not lookup hostname '%s'!", hostname);
    return false;
  }

  memcpy((char *)&sockaddr.sin_addr,(char *)remote_host->h_addr, remote_host->h_length);

  if (::connect(handle,(struct sockaddr *)&sockaddr, sizeof(sockaddr))<0)
  {
	  strcpy(errorMsg, "Could not connect to RoboRealm handle!");
    return false;
  }

	connected=true;

	return true;
}

/* close the socket handle */
void RR_API::disconnect()
{
	if (connected) closesocket(handle);
	connected=false;
}

/* Timed read from a socket */
int RR_API::read(int hSocket, unsigned char *buffer, int len)
{
	struct timeval	tim;
	fd_set fds;

	do
	{
		FD_ZERO(&fds);
		FD_SET(hSocket, &fds);

		tim.tv_sec = (timeout/1000);
		tim.tv_usec = (timeout%1000)*10;

		if (select(1024, &fds, NULL, NULL, &tim)<=0)
		{
			return(-2);
		}
	}
	while (!(FD_ISSET(hSocket, &fds)));

  return recv(hSocket, (char *)buffer, len, NULL);
}

/*
Buffered socket image read. Since we don't know how much data was read from a
previous socket operation we have to add in any previously read information
that may still be in our buffer. We detect the end of XML messages by the
</response> tag but this may require reading in part of the image data that
follows a message. Thus when reading the image data we have to move previously
read data to the front of the buffer and continuing reading in the
complete image size from that point.
*/
int RR_API::readImageData(int hSocket, unsigned char *pixels, int len)
{
  int num;

	// check if we have any information left from the previous read
	num = lastDataSize-lastDataTop;
	if (num>len)
	{
		memcpy(pixels, &buffer[lastDataTop], len);
		lastDataTop+=num;
		return num;
	}
	memcpy(pixels, &buffer[lastDataTop], num);
	len-=num;
	lastDataSize=lastDataTop=0;

	// then keep reading until we're read in the entire image length
  do
  {
    int res = read(hSocket, (unsigned char *)&pixels[num], len);
		if (res<0)
		{
			lastDataSize=lastDataTop=0;
			return -1;
		}
    num+=res;
    len-=res;
  }
  while (len>0);

  return num;
}

/*
Skips the specified length of data in case the incoming image is too big for the current
buffer. If we don't read in the image entirely then the next API statements will
not work since a large binary buffer will be before any response can be read
*/
void RR_API::skipData(int hSocket, int len)
{
	int num = lastDataSize-lastDataTop;
	lastDataSize=lastDataTop=0;
	len-=num;
	char skipBuffer[1024];
	do
	{
		int res = read(hSocket, (unsigned char *)skipBuffer, len>1024?1024:len);
		len-=res;
	}
	while (len>0);
}

/* Read's in an XML message from the RoboRealm Server. The message is always
delimited by a </response> tag. We need to keep reading in information until
this tag is seen. Sometimes this will accidentally read more than needed
into the buffer such as when the message is followed by image data. We
need to keep this information for the next readImage call.*/

int RR_API::readMessage(int hSocket, unsigned char *buffer, int len)
{
  int num=0;
	char *delimiter = "</response>";
	int top=0;
	int i;

	// read in blocks of data looking for the </response> delimiter
  do
  {
    int res = read(hSocket, (unsigned char *)&buffer[num], len);
		if (res<0)
		{
			lastDataSize=lastDataTop=0;
			return -1;
		}
		lastDataSize=num+res;
    for (i=num;i<num+res;i++)
		{
			if (buffer[i]==delimiter[top])
			{
				top++;
				if (delimiter[top]==0)
				{
					num=i+1;
					buffer[num]=0;
			    lastDataTop=num;
					return num;
				}
			}
			else
				top=0;
		}
		num+=res;
    len-=res;
  }
  while (len>0);

	lastDataTop=num;
	buffer[num]=0;
  return num;
}

/******************************************************************************/
/* API Routines */
/******************************************************************************/

/* Returns the current image dimension */
bool RR_API::getDimension(int *width, int *height)
{
	if (!connected) return false;

	char *cmd = "<request><get_dimension/></request>";

  send(handle, cmd, strlen(cmd), NULL);

  // read in variable length
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		sscanf(buffer, "<response><width>%d</width><height>%d</height></response>", width, height);
		return true;
	}

	return false;
}

/*
Returns the current processed image.
	pixels  - output - contains RGB 8 bit byte.
	width - output - contains grabbed image width
	height - output - contains image height
	len - input - maximum size of pixels to read
*/
bool RR_API::getImage(unsigned char *pixels, int *width, int *height, unsigned int len)
{
	return getImage(NULL, pixels, width, height, len, "RGB");
}

/*
Returns the named image.
	name - input - name of image to grab. Can be source, processed, or marker name.
	pixels  - output - contains RGB 8 bit byte.
	width - output - contains grabbed image width
	height - output - contains image height
	max - input - maximum size of pixels to read
	mode - input - the format of the image to be returned
*/
bool RR_API::getImage(char *name, unsigned char *pixels, int *width, int *height, unsigned int max, char *mode)
{
	unsigned int len;
	if (!connected) return false;
	if (name==NULL) name="";
	char *format;
	char ename[64];
	// escape the name for use in an XML stream
	escape(name, ename, 64);

	if ((stricmp(mode, "RGB")==0)||(stricmp(mode, "GRAY")==0)||(stricmp(mode, "BINARY")==0)||(stricmp(mode, "RED")==0)||(stricmp(mode, "GREEN")==0)||(stricmp(mode, "BLUE")==0))
		format=mode;
	else
		format="RGB";

	// create the message request
	_snprintf(buffer, 256, "<request><get_image><name>%s</name><format>%s</format></get_image></request>", ename, format);
  send(handle, buffer, strlen(buffer), NULL);

  // read in response which contains image information
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		// parse image width and height
		sscanf(buffer, "<response><length>%d</length><width>%d</width><height>%d</height></response>", &len, width, height);

		if (len>max)
		{
	    skipData(handle, len);
			return false;
		}

		// actual image data follows the message
		if ((unsigned int)readImageData(handle, (unsigned char *)pixels, len)!=len)
			return false;
		else
		{
			return true;
		}
	}

	return false;
}

HBITMAP RR_API::getBitmap(char *name, unsigned char *pixels, int *width, int *height, unsigned int max)
{
	if (getImage(name, pixels, width, height, max, "RGB"))
	{
		HBITMAP hBitmap = NULL;

		BITMAPINFOHEADER bmiHeader;
		long size = (*width) * (*height) * 3;

		bmiHeader.biWidth              = *width;
		bmiHeader.biHeight             = *height;
		bmiHeader.biSize               = sizeof(bmiHeader);
		bmiHeader.biPlanes             = 1;
		bmiHeader.biBitCount           = 24;
		bmiHeader.biCompression        = BI_RGB;
		bmiHeader.biSizeImage          = 0;
		bmiHeader.biXPelsPerMeter      = 0;
		bmiHeader.biYPelsPerMeter      = 0;
		bmiHeader.biClrUsed            = 0;
		bmiHeader.biClrImportant        = 0;

		unsigned char *bitmapPixels = NULL;

		hBitmap = CreateDIBSection( NULL, (BITMAPINFO *)&bmiHeader, DIB_RGB_COLORS, (void **)&bitmapPixels, NULL, 0);

		if (hBitmap)
		{
			// pixel data is RGB, bitmap requires BGR
			for (int i=0;i<size;i+=3)
			{
				unsigned char t = pixels[i];
				pixels[i] = pixels[i+2];
				pixels[i+2] = t;
			}
			SetBitmapBits( hBitmap, size, pixels);
		}

		return hBitmap;
	}
	else
		return NULL;
}


/*
Sets the current source image.
	pixels  - input - contains RGB 8 bit byte.
	width - input - contains grabbed image width
	height - input - contains image height
*/
bool RR_API::setImage(unsigned char *pixels, int width, int height, bool wait, char *mode)
{
	return setImage(NULL, pixels, width, height, wait, mode);
}

/*
Sets the current source image.
	name - input - the name of the image to set. Can be source or marker name
	pixels  - input - contains RGB 8 bit byte.
	width - input - contains grabbed image width
	height - input - contains image height
*/
bool RR_API::setImage(char *name, unsigned char *pixels, int width, int height, bool wait, char *mode)
{
	if (!connected) return false;
	if (name==NULL) name="";

	char ename[64];
	// escape the name for use in an XML string
	escape(name, ename, 64);

	char *format;
	unsigned int l;
	if ((stricmp(mode, "GRAY")==0)||(stricmp(mode, "RED")==0)||(stricmp(mode, "GREEN")==0)||(stricmp(mode, "BLUE")==0))
	{
		format=mode;
		l = width*height;
	}
	else
	if (stricmp(mode, "BINARY")==0)
	{
		format=mode;
		l = (width*height)>>3;
	}
	else
	{
		format=mode;
		l = width*height*3;
	}

	// setup the message request
	_snprintf(buffer, 256, "<request><set_image><source>%s</source><format>%s</format><width>%d</width><height>%d</height><wait>%s</wait></set_image></request>", ename, format, width, height, wait?"1":"");
  send(handle, buffer, strlen(buffer), NULL);

  // send the RGB triplet pixels after message
	send(handle, (char *)pixels, l, NULL);

  // read message response
	if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

bool RR_API::setCompressedImage(unsigned char *pixels, int size, bool wait)
{
	return setCompressedImage(NULL, pixels, size, wait);
}

/*
Sets the current source image from compressed bytes.
	name - input - the name of the image to set. Can be source or marker name
	pixels  - input - contains compress image bytes.
	size - input - specifies len of bytes contained in pixels
*/
bool RR_API::setCompressedImage(char *name, unsigned char *pixels, int size, bool wait)
{
	if (!connected) return false;
	if (name==NULL) name="";

	char ename[64];
	// escape the name for use in an XML string
	escape(name, ename, 64);

	// setup the message request
	_snprintf(buffer, 256, "<request><set_image><compressed>1</compressed><source>%s</source><size>%d</size><wait>%s</wait></set_image></request>", ename, size, wait?"1":"");
  send(handle, buffer, strlen(buffer), NULL);

  // send the RGB triplet pixels after message
	send(handle, (char *)pixels, size, NULL);

  // read message response
	if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Returns the value of the specified variable.
	name - input - the name of the variable to query
	result - output - contains the current value of the variable
	max - input - the maximum size of what the result can hold
*/
bool RR_API::getVariable(char *name, char *result, int max)
{
	if (!connected) return false;
	if ((name==NULL)||(name[0]==0)) return false;

	char ename[64];
	// escape the name for use in an XML string
	escape(name, ename, 64);

	result[0]=0;

	_snprintf(buffer, 256, "<request><get_variable>%s</get_variable></request>", ename);
  send(handle, buffer, strlen(buffer), NULL);

  // read in variable length
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (sscanf_s(buffer, "<response><%*[^>]>%[^<]</%*[^>]></response>", result, max)>=1)
		{
			unescape(result);
			return true;
		}
		else
		{
			return false;
		}
	}

	return false;
}

/*
Returns the value of the specified variables.
	name - input - the names of the variable to query
	result - output - contains the current values of the variables
	max - input - the maximum size of what the result can hold
*/
int RR_API::getVariables(char *names, char *results[], int len, int rows)
{
	if (!connected) return 0;
	if ((names==NULL)||(names[0]==0)) return 0;

	char ename[100];
	// escape the name for use in an XML string
	escape(names, ename, 100);

	results[0][0]=0;

	_snprintf(buffer, 256, "<request><get_variables>%s</get_variables></request>", ename);
  send(handle, buffer, strlen(buffer), NULL);

  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (strnicmp(buffer, "<response>", 10)!=0) return 0;

		int i = 10;
		int j=0;

		while (j<rows)
		{
			// read in start tag
			if (buffer[i]!='<') return 0;
			while (buffer[i]&&(buffer[i]!='>')) i++;
			if (buffer[i]!='>') return 0;
			// read in variable value
			int p=0;
			i++;
			while ((p<len)&&buffer[i]&&(buffer[i]!='<'))
				results[j][p++]=buffer[i++];
			// read in end tag
			if (buffer[i]!='<') return 0;
			while (buffer[i]&&(buffer[i]!='>')) i++;
			if (buffer[i]!='>') return 0;
			i++;
			// unescape the resulting value
			results[j][p]=0;
			unescape(results[j]);
			// continue to next variable
			j++;

			// last part of text should be the end response tag
			if (strnicmp(&buffer[i], "</response>", 11)==0) break;
		}

		return j;
	}

	return 0;
}

/*
Sets the value of the specified variable.
	name - input - the name of the variable to set
	value - input - contains the current value of the variable to be set
*/
bool RR_API::setVariable(char *name, char *value)
{
	if (!connected) return false;
	if ((name==NULL)||(name[0]==0)) return false;

	// create request message
	strcpy(buffer, "<request><set_variable><name>");
	escape(name, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</name><value>");
	escape(value, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</value></set_variable></request>");

	// send that message to RR Server
	send(handle, buffer, strlen(buffer), NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Sets the value of the specified variables.
	names - input - the name of the variable to set
	values - input - contains the current value of the variable to be set
*/
bool RR_API::setVariables(char *names[], char *values[], int num)
{
	if (!connected) return false;
	if ((names==NULL)||(values==NULL)||(names[0][0]==0)) return false;

	int j=0;
	int i;

	// create request message
	strcpy(buffer, "<request><set_variables>");
	j=strlen(buffer);
	for (i=0;(i<num);i++)
	{
		if ((j+17)>=4096) return false;
		strcpy(&buffer[j], "<variable><name>");
		j+=strlen(&buffer[j]);
		escape(names[i], &buffer[j], 4096-j);
		j+=strlen(&buffer[j]);
		if ((j+16)>=4096) return false;
		strcpy(&buffer[j], "</name><value>");
		j+=strlen(&buffer[j]);
		escape(values[i], &buffer[j], 4096-j);
		j+=strlen(&buffer[j]);
		if ((j+20)>=4096) return false;
		strcpy(&buffer[j], "</value></variable>");
		j+=strlen(&buffer[j]);
  }
	if ((j+25)>=4096) return false;
	strcpy(&buffer[j], "</set_variables></request>");
	j+=strlen(&buffer[j]);

	// send that message to RR Server
	send(handle, buffer, j, NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Deletes the specified variable
	name - input - the name of the variable to delete
*/
bool RR_API::deleteVariable(char *name)
{
	if (!connected) return false;
	if ((name==NULL)||(name[0]==0)) return false;

	char ename[64];
	// escape the name for use in an XML string
	escape(name, ename, 64);

	_snprintf(buffer, 256, "<request><delete_variable>%s</delete_variable></request>", ename);

  send(handle, buffer, strlen(buffer), NULL);

  // read in variable length
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Executes the provided image processing pipeline
	source - the XML .robo file string
*/
bool RR_API::execute(char *source)
{
	if (!connected) return false;
	if ((source==NULL)||(source[0]==0)) return false;

	// create the request message
	strcpy(buffer, "<request><execute>");
	escape(source, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</execute></request>");

	//send the string
  send(handle, buffer, strlen(buffer), NULL);

  // read in result
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Loads and executes the provided .robo file. Note that the file needs to be on the machine
running RoboRealm. This is similar to pressing the 'open' button in the
main RoboRealm dialog.
	filename - the XML .robo file to run
*/
bool RR_API::loadProgram(char *filename)
{
	if (!connected) return false;
	if ((filename==NULL)||(filename[0]==0)) return false;

	strcpy(buffer, "<request><load_program>");
	escape(filename, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</load_program></request>");

  send(handle, buffer, strlen(buffer), NULL);

  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Saves the current .robo file. This is similar to pressing the 'save' button in the
main RoboRealm dialog.

	filename - the XML .robo file to save
*/
bool RR_API::saveProgram(char *filename)
{
	if (!connected) return false;
	if ((filename==NULL)||(filename[0]==0)) return false;

	strcpy(buffer, "<request><save_program>");
	escape(filename, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</save_program></request>");

  send(handle, buffer, strlen(buffer), NULL);

  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}


/*
Returns the currently loaded .robo file
*/
bool RR_API::getProgram(char *xml, int len)
{
	if (!connected) return false;

	char *cmd = "<request><get_program></get_program></request>";

  send(handle, cmd, strlen(cmd), NULL);

  if (readMessage(handle, (unsigned char *)xml, len)>0)
	{
		if (strnicmp(xml, "<response><", 11)!=0)
			return false;
		else
		{
			memmove(xml, &xml[10], strlen(xml)-10);
			xml[strlen(xml)-21]=0;
			return true;
		}
	}

	return false;
}


/*
Loads an image into RoboRealm. Note that the image needs to exist
on the machine running RoboRealm. The image format must be one that
RoboRealm using the freeimage.dll component supports. This includes
gif, pgm, ppm, jpg, png, bmp, and tiff. This is
similar to pressing the 'load image' button in the main RoboRealm
dialog.
	name - name of the image. Can be "source" or a marker name,
	filename - the filename of the image to load
*/
bool RR_API::loadImage(char *name, char *filename)
{
	if (!connected) return false;

	if ((filename==NULL)||(filename[0]==0)) return false;
	if ((name==NULL)||(name[0]==0)) name="source";

	strcpy(buffer, "<request><load_image><filename>");
	escape(filename, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</filename><name>");
	escape(name, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</name></load_image></request>");

  send(handle, buffer, strlen(buffer), NULL);

  // read in variable length
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Saves the specified image in RoboRealm to disk. Note that the filename is relative
to the machine that is running RoboRealm. The image format must be one that
RoboRealm using the freeimage.dll component supports. This includes
gif, pgm, ppm, jpg, png, bmp, and tiff. This is
similar to pressing the 'save image' button in the main RoboRealm
dialog.
	name - name of the image. Can be "source","processed", or a marker name,
	filename - the filename of the image to save
*/
bool RR_API::saveImage(char *source, char *filename)
{
	if (!connected) return false;

	if ((filename==NULL)||(filename[0]==0)) return false;
	if ((source==NULL)||(source[0]==0)) source="processed";

	// create the save image message
	strcpy(buffer, "<request><save_image><filename>");
	escape(filename, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</filename><source>");
	escape(source, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</source></save_image></request>");

  // send it on its way
	send(handle, buffer, strlen(buffer), NULL);

  // read in any result
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Sets the current camera driver. This can be used to change the current viewing camera
to another camera installed on the same machine. Note that this is a small delay
when switching between cameras. The specified name needs only to partially match
the camera driver name seen in the dropdown picklist in the RoboRealm options dialog.
For example, specifying "Logitech" will select any installed Logitech camera including
"Logitech QuickCam PTZ".
*/
bool RR_API::setCamera(char *name)
{
	if (!connected) return false;
	if ((name==NULL)||(name[0]==0)) return false;

	strcpy(buffer, "<request><set_camera>");
	escape(name, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</set_camera></request>");

  int res = send(handle, buffer, strlen(buffer), NULL);

  // read in variable length
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Gets the current camera driver.
*/
bool RR_API::getCamera(char *name, int max)
{
	if (!connected) return false;
	if ((name==NULL)||(name[0]==0)) return false;

	strcpy(buffer, "<request><get_camera></get_camera></request>");

  int res = send(handle, buffer, strlen(buffer), NULL);

  // read in variable length
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (sscanf_s(buffer, "<response>%[^<]</response>", name, max)>=1)
		{
			unescape(name);
			return true;
		}
		else
		{
			name[0]=0;
			return false;
		}
	}

	return false;
}

/*
This routine provides a way to stop processing incoming video. Some image processing
tasks can be very CPU intensive and you may only want to enable processing when
required but otherwise not process any incoming images to release the CPU for other
tasks. The run mode can also be used to processing individual frames or only run
the image processing pipeline for a short period. This is similar to pressing the
"run" button in the main RoboRealm dialog.
	mode - can be toggle, on, off, once, or a number of frames to process
	*/
bool RR_API::run(char *mode)
{
	if (!connected) return false;
	if ((mode==NULL)||(mode[0]==0)) return false;

	strcpy(buffer, "<request><run>");
	escape(mode, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</run></request>");

  send(handle, buffer, strlen(buffer), NULL);

  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
There is often a need to pause your own Robot Controller program to wait for
RoboRealm to complete its task. The eaisest way to accomplish this is to wait
on a specific variable that is set to a specific value by RoboRealm. Using the
waitVariable routine you can pause processing and then continue when a variable
changes within RoboRealm.
	name - name of the variable to wait for
	value - the value of that variable which will cancel the wait
	timeout - the maximum time to wait for the variable value to be set
*/
bool RR_API::waitVariable(char *name, char *value, int timeout)
{
	if (!connected) return false;
	if ((name==NULL)||(name[0]==0)) return false;

	strcpy(buffer, "<request><wait_variable><name>");
	escape(name, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</name><value>");
	escape(value, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</value><timeout>");
	itoa(timeout, &buffer[strlen(buffer)], 10);
	strcat(buffer, "</timeout></wait_variable></request>");

	this->timeout=timeout;
	if (timeout==0) timeout=100000000;

  send(handle, buffer, strlen(buffer), NULL);

  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		this->timeout=DEFAULT_TIMEOUT;
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	this->timeout=DEFAULT_TIMEOUT;
	return false;
}

/*
If you are rapdily grabbing images you will need to wait inbetween each
get_image for a new image to be grabbed from the video camera. The wait_image
request ensures that a new image is available to grab. Without this routine
you may be grabbing the same image more than once.
*/
bool RR_API::waitImage(int timeout)
{
	if (!connected) return false;

	sprintf(buffer, "<request><wait_image><timeout>%d</timeout></wait_image></request>", timeout);

  send(handle, buffer, strlen(buffer), NULL);

  // read in variable length
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/* If you are running RoboRealm on the same machine as your API program you can use
this routine to start RoboRealm if it is not already running.
	filename - the path to RoboRealm on your machine
*/
bool RR_API::open(char *filename, int port)
{
	char buffer[256];

	if (port!=6060)
		sprintf(buffer, "RoboRealm_Server_Event_%d", port);
	else
		strcpy(buffer, "RoboRealm_Server_Event");

	HANDLE serverReady = CreateEvent(
		NULL,					//use default security attributes
		TRUE,                  //event will be auto reset
		FALSE,                  //initial state is non-signalled
		buffer);

	// for Unicode use
	/*
	HANDLE serverReady = CreateEvent(
        NULL,                    //use default security attributes
        TRUE,                  //event will be auto reset
        FALSE,                  //initial state is non-signalled
        L"RoboRealm_Server_Event");
*/


	int res = WaitForSingleObject(serverReady, 100);
	if (res==WAIT_OBJECT_0) return true;

	PROCESS_INFORMATION piProcInfo;
	STARTUPINFO siStartInfo;

  ZeroMemory( &piProcInfo, sizeof(piProcInfo));
  ZeroMemory( &siStartInfo, sizeof(siStartInfo));

  siStartInfo.cb = sizeof(STARTUPINFO);
  siStartInfo.lpReserved = NULL;

	// Unicode version
/*
    unsigned short int name[MAX_PATH];
  ::MultiByteToWideChar(CP_ACP, 0, filename, -1, name, MAX_PATH);
  if (!CreateProcess(name,
    name,
	  NULL,          // process security attributes
	  NULL,          // primary thread security attributes
	  TRUE,          // handles are inherited
	  0,             // creation flags
	  NULL,          // use parent's environment
	  NULL,          // use parent's current directory

	  &siStartInfo,  // STARTUPINFO pointer
	  &piProcInfo))  // receives PROCESS_INFORMATION
    return false;
	*/

	_snprintf(buffer, 256, "\"%s\" -api_port %d", filename, port);
  /* Create the child process. */
	if (!CreateProcess(filename,
	  buffer,       /* command line                       */
	  NULL,          /* process security attributes        */
	  NULL,          /* primary thread security attributes */
	  TRUE,          /* handles are inherited              */
	  0,             /* creation flags                     */
	  NULL,          /* use parent's environment           */
	  NULL,          /* use parent's current directory     */

	  &siStartInfo,  /* STARTUPINFO pointer                */
	  &piProcInfo))  /* receives PROCESS_INFORMATION       */
    return false;
  else
	{
		//need to wait to ensure that process has started. We do this by
		// waiting for the roborealm server event for up to 30 seconds!!
		int res = WaitForSingleObject(serverReady, 30000);
		if (res==WAIT_OBJECT_0)
	  	return true;
		else
			return false;
	}
}


/* Closes the roborealm application nicely. */
bool RR_API::close()
{
	if (!connected) return false;

	strcpy(buffer, "<request><close></close></request>");

  send(handle, buffer, strlen(buffer), NULL);

  // read in variable length
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	disconnect();

	return false;
}

//////////////////////////////////// Basic Image Load/Save routines ////////////////////////

// Utility routine to save a basic PPM
int RR_API::savePPM(char *filename, unsigned char *buffer, int width, int height)
{
  FILE *fp;
  int num;
  int len;
  int length=width*height*3;

  if ((fp=fopen(filename,"wb"))!=NULL)
  {
    fprintf(fp, "P6\n%d %d\n255\n", width, height);

    for (len=0;len<length;len+=num)
    {
      if ((num=(length-len))>4096)
        num=4096;
      num=fwrite(&buffer[len], 1, num, fp);
    }

		fclose(fp);
  }
  return len;
}

unsigned char *RR_API::readLine(FILE *fp, unsigned char *buffer)
{
  while (!feof(fp))
  {
    fscanf(fp, "%[^\n]\n", buffer);
    if (buffer[0]!='#')
      return buffer;
  }
  return NULL;
}

// Utility routine to load a basic PPM. Note that this routine does NOT handle
// comments and is only included as a quick example.
int RR_API::loadPPM(char *filename, unsigned char *buffer, int *width, int *height, int max)
{
  FILE *fp;
  int len, num, w,h;

  if ((fp=fopen(filename,"rb"))!=NULL)
	{
    readLine(fp, buffer);
		if (strcmp((char *)buffer, "P6")!=0)
		{
			printf("Illegal format!\n");
			fclose(fp);
			return -1;
		}

    readLine(fp, buffer);
		sscanf((char *)buffer, "%d %d", &w, &h);

		*width=w;
		*height=h;

    readLine(fp, buffer);
		if (strcmp((char *)buffer, "255")!=0)
		{
			printf("Illegal format!\n");
			fclose(fp);
			return -1;
		}

		for (len=0;(len<w*h*3)&&(len<max);len+=num)
		{
			if (len+65535>max) num=max-len; else num=65535;
			num=fread(&buffer[len], 1, num, fp);
			if (num==0) break;
		}

		fclose(fp);

		return 1;
	}

	return -1;
}

/*
Sets the value of the specified parameter.
	module - input - the name of the module which contains the parameter
	module_number - input - module count in case you have more than one of the same module
	name - input - the name of the variable to set
	value - input - contains the current value of the variable to be set
*/
bool RR_API::setParameter(char *module, int count, char *name, char *value)
{
	if (!connected) return false;
	if ((module==NULL)||(module[0]==0)) return false;
	if ((name==NULL)||(name[0]==0)) return false;

	// create request message
	strcpy(buffer, "<request><set_parameter><module>");
	escape(module, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</module><module_number>");
	itoa(count, &buffer[strlen(buffer)], 10);
	strcat(buffer, "</module_number><name>");
	escape(name, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</name><value>");
	escape(value, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</value></set_parameter></request>");

	// send that message to RR Server
	send(handle, buffer, strlen(buffer), NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Returns the value of the specified parameter.
	module - input - the name of the module which contains the parameter
	module_number - input - module count in case you have more than one of the same module
	name - input - the name of the parameter to query
	result - output - contains the current value of the parameter
	max - input - the maximum size of what the result can hold
*/
bool RR_API::getParameter(char *module, int count, char *name, char *result, int max)
{
	if (!connected) return false;
	if ((name==NULL)||(name[0]==0)) return false;

	strcpy(buffer, "<request><get_parameter><module>");
	escape(module, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</module><module_number>");
	itoa(count, &buffer[strlen(buffer)], 10);
	strcat(buffer, "</module_number><name>");
	escape(name, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</name></get_parameter></request>");

	result[0]=0;

  send(handle, buffer, strlen(buffer), NULL);

  // read in variable length
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (sscanf_s(buffer, "<response><%*[^>]>%[^<]</%*[^>]></response>", result, max)>=1)
		{
			unescape(result);
			return true;
		}
		else
			return false;
	}

	return false;
}

/*
Minimizes the RoboRealm interface window
*/
bool RR_API::minimizeWindow()
{
	if (!connected) return false;

	char *msg = "<request><interface><command>minimize</command></interface></request>";

	// send that message to RR Server
	send(handle, msg, strlen(msg), NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Maximize the RoboRealm interface window
*/
bool RR_API::maximizeWindow()
{
	if (!connected) return false;

	char *msg = "<request><interface><command>maximize</command></interface></request>";

	// send that message to RR Server
	send(handle, msg, strlen(msg), NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Moves the RoboRealm interface window
*/
bool RR_API::moveWindow(int x, int y)
{
	if (!connected) return false;

	_snprintf(buffer, 256, "<request><interface><command>move</command><x_position>%d</x_position><y_position>%d</y_position></interface></request>", x, y);

	// send that message to RR Server
	send(handle, buffer, strlen(buffer), NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Resizes the RoboRealm interface window
*/
bool RR_API::resizeWindow(int width, int height)
{
	if (!connected) return false;

	_snprintf(buffer, 256, "<request><interface><command>resize</command><width>%d</width><height>%d</height></interface></request>", width, height);

	// send that message to RR Server
	send(handle, buffer, strlen(buffer), NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Moves and resizes the RoboRealm interface window
*/
bool RR_API::positionWindow(int x, int y, int width, int height)
{
	if (!connected) return false;

	char buffer[256];

	_snprintf(buffer, 256, "<request><interface><command>position</command><x_position>%d</x_position><y_position>%d</y_position><width>%d</width><height>%d</height></interface></request>", x, y, width, height);

	// send that message to RR Server
	send(handle, buffer, strlen(buffer), NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Shows or hides the GUI interface (Commercial version only)
*/
bool RR_API::showWindow(bool show)
{
	if (!connected) return false;

	char buffer[256];

	if (show)
		strcpy(buffer, "<request><interface><command>display</command></interface></request>");
	else
		strcpy(buffer, "<request><interface><command>faceless</command></interface></request>");

	// send that message to RR Server
	send(handle, buffer, strlen(buffer), NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Changes the currently active camera format settings to the specified values. For example, you
can use this function call to change the size of the image from 320x240 to 640x480 or to change
from one encoding RGB to another like YUV depending on the supported compression formats for
your camera.
*/
bool RR_API::setCameraFormat(int width, int height, int frameRate, char compression[4])
{
	if (!connected) return false;

	char buffer[256];
	char tmp[64];

	strcpy(buffer, "<request><set_camera_format>");
	if ((width>0)&&(height>0))
	{
		_snprintf(tmp, 256, "<width>%d</width><height>%d</height>", width, height);
		strcat(buffer, tmp);
	}
	if (frameRate>0)
	{
		_snprintf(tmp, 256, "<frame_rate>%d</frame_rate>", frameRate);
		strcat(buffer, tmp);
	}
	if (compression[0])
	{
		_snprintf(tmp, 256, "<compression>%s</compression>", compression);
		strcat(buffer, tmp);
	}
	strcat(buffer, "</set_camera_format></request>");

	// send that message to RR Server
	send(handle, buffer, strlen(buffer), NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Returns the current camera format. Note that if you do not need frameRate and/or compression
either specify 0 or NULL for those arguments or simply don't specify them and the defaults
will ensure they do not get set.
*/
bool RR_API::getCameraFormat(int *width, int *height, int *frameRate, char compression[4])
{
	if (!connected) return false;

	char *buffer = "<request><get_camera_format></get_camera_format></request>";
	char reply[256];

  send(handle, buffer, strlen(buffer), NULL);

  // read in variable length
  if (readMessage(handle, (unsigned char *)reply, sizeof(reply))>0)
	{
		char *node;

		if (width!=NULL)
		{
			node = strstr(reply, "<width>");
			if (node!=NULL)
				sscanf(node, "<width>%d</width>", width);
			else
				*width=0;
		}

		if (height!=NULL)
		{
			node = strstr(reply, "<height>");
			if (node!=NULL)
				sscanf(node, "<height>%d</height>", height);
			else
				*height=0;
		}

		if (frameRate!=NULL)
		{
			node = strstr(reply, "<frame_rate>");
			if (node!=NULL)
				sscanf(node, "<frame_rate>%d</frame_rate>", frameRate);
			else
				*frameRate=0;
		}

		if (compression!=NULL)
		{
			node = strstr(reply, "<compression>");
			if (node!=NULL)
			{
				if (sscanf_s(node, "<compression>%[^<]</compression>", compression, 4)>=1)
					return true;
				else
				{
					compression[0]=0;
					return false;
				}
			}
			else
				compression[0]=0;
		}

		return true;
	}

	return false;
}

/*

	Parses the tagset
		<property><value>x</value><min>x</min><max>x</max></property>
	which is returned for each property
*/

int RR_API::parseProperyTagset(char *buffer, int i, char *name, int *value, int *min, int *max, int *automatic)
{
	// prepare to look for <property_name> tag
	char startTag[64], endTag[64];
	_snprintf(startTag, 64, "<%s>", name);
	_snprintf(endTag, 64, "</%s>", name);

	// find the start tag
	char *node = strstr(&buffer[i], startTag);
	if (node!=NULL)
	{
		char *tmp;
		// also find the end tag to ensure that if a value, min, or max is missing we don't
		// pickup the next tagsets value.
		char *end = strstr(node, endTag);

		// grab the value
		if (((tmp = strstr(node, "<value>"))!=NULL)&&(tmp<end))
			sscanf(tmp, "<value>%d</value>", value);
		else
			*value=0;

		// grab the min
		if (((tmp = strstr(node, "<min>"))!=NULL)&&(tmp<end))
			sscanf(tmp, "<min>%d</min>", min);
		else
			*min=0;

		// grab the max
		if (((tmp = strstr(node, "<max>"))!=NULL)&&(tmp<end))
			sscanf(tmp, "<max>%d</max>", max);
		else
			*max=0;

		// grab the if there is an auto switch
		if (((tmp = strstr(node, "<auto>"))!=NULL)&&(tmp<end))
			sscanf(tmp, "<auto>%d</auto>", automatic);
		else
			*automatic=0;

		// return the index for the next tagset
		return i+((long)end-(long)node)+strlen(endTag);
	}
	else
	{
		*value=0;
		*min=0;
		*max=0;
		*automatic=0;

		return i;
	}
}

/*
Returns the requested camera properties. If no properties are specified in "names" then
all properties will be returned up to "num" amount.
*/
bool RR_API::getCameraProperties(char **names, int *values, int *min, int *max, int *automatic, int num)
{
	int i;
	bool assumeAll;

	strcpy(buffer, "<request><get_camera_properties>");
	for (i=0;i<num;i++)
	{
		if (names[i][0])
		{
			strcat(buffer, "<");
			strcat(buffer, names[i]);
			strcat(buffer, "></");
			strcat(buffer, names[i]);
			strcat(buffer, ">");
		}
	}
	// check if any properties have been requested ... if not send all back
	if (!buffer[32]) assumeAll=true; else assumeAll=false;
	strcat(buffer, "</get_camera_properties></request>");

	// send the request to RoboRealm
  send(handle, buffer, strlen(buffer), NULL);

  // read in resulting XML string
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (strnicmp(buffer, "<response>", 10)!=0) return false;

		// if no name is set then assume all properties
		if (assumeAll)
		{
			int i = 10;
			int j = 0;

			while (j<num)
			{
				int p=0;
				int hold = i;
				// read in start tag

				if (buffer[i]!='<') return false;
				i++;
				while ((p<64)&&buffer[i]&&(buffer[i]!='>'))
					names[j][p++]=buffer[i++];
				names[j][p]=0;

				// parse the tagset now that we know the name
				i=parseProperyTagset(buffer, hold, names[j], &values[j], &min[j], &max[j], &automatic[j]);

				// continue to next variable
				j++;

				// last part of text should be the end response tag
				if (strnicmp(&buffer[i], "</response>", 11)==0) break;
			}

			if (j<num)
				names[j][0]=0;
		}
		else
		{
			// scan for each requested property
			for (i=0;i<num;i++)
			{
				if (names[i][0])
				{
					// only process names < 60 characters (failsafe)
					int len = strlen(names[i]);
					if (len<60)
					{
						// parse the tagset
						parseProperyTagset(buffer, 0, names[i], &values[i], &min[i], &max[i], &automatic[i]);
					}
				}
			}
		}
	}

	return true;
}

/*
Sets the camera properties such as brightness, contrast, etc. using DirectX
*/

bool RR_API::setCameraProperties(char **names, int *values, int *automatic, int num)
{
	char buffer[1024];
	int i;

	strcpy(buffer, "<request><set_camera_properties>");
	for (i=0;i<num;i++)
	{
		if (names[i][0])
		{
			char value[64];
			_snprintf(value, 64, "<%s><value>%d</value><auto>%d</auto></%s>", names[i], values[i], automatic[i], names[i]);
			strcat(buffer, value);
		}
	}

	strcat(buffer, "</set_camera_properties></request>");

	// send the request to RoboRealm
  send(handle, buffer, strlen(buffer), NULL);

  // read in resulting XML string
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (strstr(buffer, ">error</")!=NULL)
			return false;
		else
			return true;
	}

	return false;
}

bool RR_API::getVersion(char *version)
{
	if (!connected) return false;

	char *request = "<request><version></version></request>";

	// send that message to RR Server
	int res = send(handle, request, strlen(request), NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		sscanf(buffer, "<response>%[^<]</response>", version);
		return true;
	}

	return false;

}

/*
Delete a particular module in the pipeline
	module - input - the name of the module which contains the parameter
	module_number - input - module count in case you have more than one of the same module
*/
bool RR_API::deleteModule(char *module, int count)
{
	if (!connected) return false;
	if ((module==NULL)||(module[0]==0)) return false;

	// create request message
	strcpy(buffer, "<request><delete_module><module>");
	escape(module, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</module><module_number>");
	itoa(count, &buffer[strlen(buffer)], 10);
	strcat(buffer, "</module_number></delete_module></request>");

	// send that message to RR Server
	send(handle, buffer, strlen(buffer), NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}

/*
Toggles disable/enable of a particular module in the pipeline
	module - input - the name of the module which contains the parameter
	module_number - input - module count in case you have more than one of the same module
*/
bool RR_API::toggleModule(char *module, int count)
{
	if (!connected) return false;
	if ((module==NULL)||(module[0]==0)) return false;

	// create request message
	strcpy(buffer, "<request><toggle_module><module>");
	escape(module, &buffer[strlen(buffer)], 4096-strlen(buffer));
	strcat(buffer, "</module><module_number>");
	itoa(count, &buffer[strlen(buffer)], 10);
	strcat(buffer, "</module_number></toggle_module></request>");

	// send that message to RR Server
	send(handle, buffer, strlen(buffer), NULL);

  // read in confirmation
  if (readMessage(handle, (unsigned char *)buffer, MAX_BUFFER_SIZE)>0)
	{
		if (stricmp(buffer, "<response>ok</response>")!=0)
			return false;
		else
			return true;
	}

	return false;
}
