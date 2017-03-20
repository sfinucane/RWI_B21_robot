
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/tcx/hash.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:48:15 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: hash.c,v $
 * Revision 1.1  2002/09/14 15:48:15  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1997/03/11 17:16:47  tyson
 * added IR simulation and other work
 *
 * Revision 1.1.1.1  1996/09/22 16:46:01  rhino
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


/******************************************************************************
*
* PROJECT: Carnegie Mellon Planetary Rover Project
*          Task Control Architecture 
* 
* (c) Copyright 1991 Christopher Fedor and Reid Simmons.  All rights reserved.
* 
* MODULE: hash
*
* FILE: hash.c
*
* ABSTRACT:
* 
* Generic hash table abstract data type.
*
* REVISION HISTORY
*
*  6-Apr-90 Christopher Fedor, School of Computer Science, CMU
* Revised to Software Standards.
*
* 17-Nov-89 Reid Simmons, School of Computer Science, CMU
* Added function for printing hash table stats.
*
* 10-Feb-89 Christopher Fedor, School of Computer Science, CMU
* Created.
*
******************************************************************************/

#ifdef VMS
#include "vms.h"                               
#endif

#ifdef VXWORKS
#include "/usr/vxworks/vx5.0.2b/h/vxWorks.h"
#include "/usr/vxworks/vx5.0.2b/h/stdioLib.h"
#else
#include "stdio.h" 
#endif

#include "hash.h"


/******************************************************************************
*
* FUNCTION: HASH_TABLE_PTR hashTableCreate(size, hashFunc, eqFunc)
*
* DESCRIPTION:
* Item is the element to be stored.
*
* HashFunc(Key)  
* - returns a hash value of the key.
* - this value will be modded with table size when (key, item) pair
*   is added to the table.
*
* EqualityFunc(Key1, Key2) 
* - returns 1 if the key values Key1 and Key2 are equal, 0 otherwise.
*
* INPUTS: 
* int size;
* int (*hashFunc)();
* int (*eqFunc)();
*
* OUTPUTS: HASH_TABLE_PTR
*
******************************************************************************/

HASH_TABLE_PTR hashTableCreate(size, hashFunc, eqFunc)
int size;
int (*hashFunc)();
int (*eqFunc)();
{
  int i;
  HASH_ELEM_PTR *table;
  HASH_TABLE_PTR hashTable;

  hashTable = (HASH_TABLE_TYPE *)malloc(sizeof(HASH_TABLE_TYPE));
  table = (HASH_ELEM_PTR *)malloc(sizeof(HASH_ELEM_PTR)*size);

  for(i=0;i < size;i++)
    table[i] = NULL;

  hashTable->size = size;
  hashTable->hashFunc = hashFunc;
  hashTable->eqFunc = eqFunc;
  hashTable->table = table;

  return hashTable;
}


/******************************************************************************
*
* FUNCTION: HASH_ELEM_PTR findElement(eq, list, key)
*
* DESCRIPTION: 
* Iterate through the element list returning the element of the list whose
* key value is "eq" to the given key. NULL is returned if no match is found.
*
* INPUTS:
* int (*eq)();
* HASH_ELEM_PTR list;
* char *key;
*
* OUTPUTS: HASH_ELEM_PTR
*
******************************************************************************/

HASH_ELEM_PTR findElement(eq, list, key)
int (*eq)();
HASH_ELEM_PTR list;
char *key;
{
  HASH_ELEM_PTR tmp;

  tmp = list;
  while (tmp) {
    if ((*eq)(tmp->key, key))
      return tmp;
    tmp = tmp->next;
  }
  
  return NULL;
}


/******************************************************************************
*
* FUNCTION: char *hashTableFind(key, table)
*
* DESCRIPTION: The item is returned or NULL if not found.
*
* INPUTS: 
* char *key;
* HASH_TABLE_PTR table;
*
* OUTPUTS: char *
*
******************************************************************************/

char *hashTableFind(key, table)
char *key;
HASH_TABLE_PTR table;
{
  HASH_ELEM_PTR tmp;
  int hash, location;

  hash = (*table->hashFunc)(key);
  location = hash % table->size;

  tmp = table->table[location];
  if (tmp) {
    tmp = findElement(table->eqFunc, tmp, key);
    if (tmp)
      return(tmp->data);
    else
      return NULL;
  }
  else
    return NULL;
}


/******************************************************************************
*
* FUNCTION: char *hashTableInsert(key, keySize, item, table)
*
* DESCRIPTION:
* The key value is copied for future lookups.
* The old Item will be returned if the new item replaces 
* one of the same key value. 
* - A warning is also issued - the warning may go away soon.
*
* INPUTS: 
* char *key;
* int keySize;
* char *item;
* HASH_TABLE_PTR table;
*
* OUTPUTS: char * (any item that was already stored under this key or NULL)
*
* NOTES:
* It is common to use a string key be sure that keySize is then
* equal to strlen(key)+1 so that the NULL terminator is also stored.
*
******************************************************************************/

char *hashTableInsert(key, keySize, item, table)
char *key;
int keySize;
char *item;
HASH_TABLE_PTR table;
{
  char *oldData;
  int hash, location;
  HASH_ELEM_PTR tmp, element;

  hash = (*table->hashFunc)(key);
  location = hash % table->size;

  tmp = table->table[location];
  if (tmp) {
    tmp = findElement(table->eqFunc, tmp, key);
    if (tmp) {
      /* replace item with new information */
      oldData = tmp->data;
      tmp->data = item;
      /*
      fprintf(stderr, 
      "hashTableInsert: WARNING: Data of same key value replaced.\n");
      */
      return oldData;
    }
  }
      
  element = (HASH_ELEM_TYPE *)malloc(sizeof(HASH_ELEM_TYPE));
  element->key = (char *)malloc(keySize);
  bcopy(key, element->key, keySize);
  element->data = item;
  element->next = table->table[location];
  table->table[location] = element;

  return NULL;
}


/******************************************************************************
*
* FUNCTION: char *hashTableRemove(key, table)
*
* DESCRIPTION: The item stored with this key value is returned.
*
* INPUTS: 
* char *key;
* HASH_TABLE_PTR table;
*
* OUTPUTS: char *
*
******************************************************************************/

char *hashTableRemove(key, table)
char *key;
HASH_TABLE_PTR table;
{
  int (*eq)();
  char *oldData;
  int hash, location;
  HASH_ELEM_PTR previous, current;

  hash = (*table->hashFunc)(key);
  location = hash % table->size;

  eq = table->eqFunc;

  previous = table->table[location];
  if (!previous)
    return NULL;

  if ((*eq)(previous->key, key)) {
    table->table[location] = previous->next;
    oldData = previous->data;
    free(previous->key);
    free(previous);
    return oldData;
  }
  current = previous->next;
  while (current) {
    if ((*eq)(current->key, key)) {
      oldData = current->data;
      previous->next = current->next;
      free(current->key);
      free(current);
      return oldData;
    }
    previous = current;
    current = current->next;
  }

  return NULL;
}


/******************************************************************************
*
* FUNCTION: void hashTableIterate(iterFunc, table)
*
* DESCRIPTION:
* interFunc(key, data)
* char *key, *data;
*
* iterFunc
*  - takes two arguments, a pointer to Key information and a pointer to Data.
*
* hashTableIterate will call the function on all of its elements stoping
* when the list is finished or when iterFunc returns 0 (ie FALSE)
*
* INPUTS: 
* int (*iterFunc)();
* HASH_TABLE_PTR table;
*
* OUTPUTS: void.
*
******************************************************************************/

void hashTableIterate(iterFunc, table)
int (*iterFunc)();
HASH_TABLE_PTR table;
{
  int i;
  HASH_ELEM_PTR tmp;

  for (i=0;i < table->size;i++) {
    tmp = table->table[i];
    while (tmp){
      if (!(*iterFunc)(tmp->key, tmp->data))
	return;
      tmp = tmp->next;
    }
  }
}


/******************************************************************************
*
* FUNCTION: void hashTableStats(hashTable)
*
* DESCRIPTION: Calculates and displays hash table stats.
*
* INPUTS: HASH_TABLE_PTR hashTable;
*
* OUTPUTS: void. Results displayed.
*
******************************************************************************/

void hashTableStats(hashTable)
HASH_TABLE_PTR hashTable;
{
  HASH_ELEM_PTR elem;
  int i, num=0, max=0, full=0, length;

  for (i=0;i < hashTable->size; i++) {
    elem = hashTable->table[i];
    if (elem) {
      full++;
      length = 0;
      while (elem) {
	num++;
	length++;
	elem = elem->next;
      }
      if (length > max) 
	max = length;
    }
  }
  fprintf(stderr, 
	  "hashTableStats: Has %d elements in %d slots; maximum list is %d\n",
	  num, full, max);
}
