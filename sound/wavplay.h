/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Welcome!
 *****
 ***** This file is part of the robot control software provided
 ***** by Real World Interface Inc.
 *****
 ***** All copyrights are by Real World Interface Inc., Carnegie
 ***** Mellon University, and the University of Bonn, Germany.
 ***** Removing the copyright information is illegal. Please read
 ***** and make sure you understand the disclaimer below.
 *****
 ***** Contact tyson@rwii.com if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                      Tyson D. Sawyer and Sebastian Thrun
 *****
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED
 ***** BY APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING
 ***** THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM
 ***** "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR
 ***** IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 ***** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE
 ***** ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
 ***** IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME
 ***** THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO
 ***** LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES
 ***** SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM
 ***** TO OPERATE WITH ANY OTHER PROGRAMS OR FAILURE TO CONTROL A
 ***** PHYSICAL DEVICE OF ANY TYPE), EVEN IF SUCH HOLDER OR OTHER
 ***** PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/sound/wavplay.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:41:10 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 *  $Log: wavplay.h,v $
 *  Revision 1.1  2002/09/14 15:41:10  rstone
 *  *** empty log message ***
 *
 *  Revision 1.1  1997/03/13 16:21:12  haehnel
 *  SOUND - include soundblaster, cd and speechboard
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#define PLAYER
#define AUDIO "/dev/dsp"

typedef unsigned long DWORD;
typedef unsigned short WORD;

typedef struct {		/* header for WAV-Files */
	char main_chunk[4];	/* 'RIFF' */
	DWORD length;		/* length of file */
	char chunk_type[4];	/* 'WAVE' */
	char sub_chunk[4];	/* 'fmt' */
	DWORD length_chunk;	/* length sub_chunk, always 16 bytes */
	WORD format;		/* always 1 = PCM-Code */
	WORD modus;		/* 1 = Mono, 2 = Stereo */
	DWORD sample_fq;	/* Sample Freq */
	DWORD byte_p_sec;	/* Data per sec */
	WORD byte_p_spl;	/* bytes per sample, 1=8 bit, 2=16 bit (mono)
						     2=8 bit, 4=16 bit (stereo) */
	WORD bit_p_spl;		/* bits per sample, 8, 12, 16 */
	char data_chunk[4];	/* 'data' */
	DWORD data_length;	/* length of data */
} wave_header;

typedef struct {		/* options set */
	DWORD speed;		/* -s xxxxx */
	int force_speed;
	int stereo;		/* -S */
	int force_stereo;
	DWORD sample_size;	/* -b xx */
	int force_sample_size;
	float time_limit;	/* -t xxx */
	DWORD lower_border;
	DWORD upper_border;	/* for xplay */
} header_data;


