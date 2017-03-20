
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliStack.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:25:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: colliStack.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:04  rhino
 * General reorganization of the directories/repository, fusion with the
 * RWI software.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include "collisionIntern.h"




int actualStackSize = 0;

struct node *head;
struct node *bottom;


/* This procedure initializes the whole stack-structure */
void initStack()
{
    head = (struct node *) malloc(sizeof(struct node));
    if (!head)
      { 
	printf("initStack : Nicht genug Speicher vorhanden");
	exit(3);
      }

    bottom = (struct node *) malloc(sizeof(struct node));
    if (!bottom)
      { 
	printf("initStack : Nicht genug Speicher vorhanden");
	exit(3);
      }

    head->next = bottom;
    head->prev = head;
    bottom->next = bottom;
    bottom->prev = head;
}

/* The following function inserts a value of type itemType at the top of
   the stack and increases the actualStackSize. If actualStackSize exceeds
   ACTUAL_MODE->stack_size then we delete the value at the bottom of
   the stack                                                               */
void push(struct itemType v)
{
    struct node *t = (struct node *) malloc(sizeof(struct node));
    if (!t)
      { 
	printf("push : Nicht mehr genug Speicher vorhanden");
	exit(3);
      }

    t->key.pos.x = v.pos.x;
    t->key.pos.y = v.pos.y;
    t->key.rot   = v.rot;
    t->next = head->next;
    head->next = t;
    t->next->prev = t;
    t->prev = head;
    actualStackSize += 1;
    
    if(actualStackSize > ACTUAL_MODE->stack_size)
      {
	deleteBottom();
	actualStackSize -= 1;
      }
	
}


/* This function pops the value at the top of the stack */
struct itemType pop()
{
    if(!isempty())
      {
	struct itemType tmp;
	struct node *t = head->next;
	
	head->next = t->next;
	t->next->prev = head;
	tmp.pos.x = t->key.pos.x;
	tmp.pos.y = t->key.pos.y;
	tmp.rot   = t->key.rot;
	free((void *)t);
	
	actualStackSize -= 1;
	return tmp;
      }
    else
      {
	printf("pop: Der Stack ist bereits leer \n");
	exit(1);
      }
} 


/* Service-function */
/* Deletes the value at the bottom of the stack */
void deleteBottom()
{
    struct node *t = bottom->prev;
    
    bottom->prev = t->prev;
    t->prev->next = bottom;
    free((void *)t);
}
    

/* Tests if the stack is empty */
int isempty()
{
    return (head->next == bottom);
}


void printStack()
{   
    struct node *tmpNode = head->next;
    
    printf("\n Stackgroesse: %i \n",actualStackSize);
    while(tmpNode != bottom)
	{
	    printf("(%f , %f, %f) \n",tmpNode->key.pos.x,tmpNode->key.pos.y,
		   tmpNode->key.rot);
	    tmpNode = tmpNode->next;
	}
    printf("\n");
}














