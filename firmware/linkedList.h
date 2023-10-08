#ifndef LINKED_LIST
#define LINKED_LIST

#include <Arduino.h>

struct listItem {
  void * item;
  void * next;
  void * prev;
};

struct linkedList {
  listItem * first;
  listItem * last;
};

linkedList * newList() {
  linkedList* list = new linkedList;
  list->first = NULL;
  list->last = NULL;
  return list;
}

void listAppend(linkedList* list, void* item) {
  listItem* newItem = new listItem;
  newItem->item = item;
  newItem->next = NULL;
  if(list->last) {
    list->last->next = (void*) newItem;
    newItem->prev = list->last;
    list->last = newItem;
  } else {
    newItem->prev = NULL;
    list->first = newItem;
    list->last = newItem;
  }
}

void* deleteItem(linkedList* list, void* item) {
  if(list->first) {
    listItem* iter = list->first;
    do {
      if(iter->item == item) {
        void* retval = iter->item;
        if(iter->prev) {
          ((listItem*)iter->prev)->next = iter->next;
          if(!iter->next) {
            list->last = (listItem*) iter->prev;
          } else {
            ((listItem*)iter->next)->prev = iter->prev;
          }
        } else if(iter->next) {
          list->first = (listItem*) iter->next;
          ((listItem*)iter->next)->prev = NULL;
        } else {
          list->first = NULL;
          list->last = NULL;
        }
        delete iter;
        return retval;
      }
      iter = (listItem*) iter->next;
    } while(iter);
  }
  return NULL;
}

void* deleteIdx(linkedList* list, int idx) {
  int count = 0;
  listItem* iter = list->first;
  while(iter) {
    if(count == idx) {
      void* item = iter->item;
      return deleteItem(list, item);
    }
    iter = (listItem*) iter->next;
    count++;
  }
  return NULL;
}

uint32_t listLength(linkedList* listPtr) {
  if(listPtr) {
    if(listPtr->first) {
      listItem* ptr = listPtr->first;
      uint32_t count = 1;
      while(ptr->next) {
        count++;
        ptr = (listItem*) ptr->next;
      }
      return count;
    }
    return 0;
  } 
  return 0;
}


void printList(linkedList* list) {
  listItem * iter = list->first;

  Serial.print("List: ");
  while(iter) {
    Serial.print(*(int*) iter->item);
    Serial.print(" ");
    iter = (listItem*) iter->next;
  }
  Serial.print("\n");
}

void clearList(linkedList* list) {
  while(listLength(list)) {
    deleteIdx(list, 0);
  }
}

void listTest() {
  linkedList* list = newList();
  printList(list);
  int* int1 = new int;
  int* int2 = new int;
  int* int3 = new int;
  int* int4 = new int;
  *int1 = 1;
  *int2 = 2;
  *int3 = 3;
  *int4 = 4;

  listAppend(list, int1);
  listAppend(list, int2);
  listAppend(list, int3);  
  listAppend(list, int4);
  printList(list);

  delay(1000);
  Serial.print("Delete Item\n");
  deleteItem(list, int2);
  printList(list);
  delay(1000);
  Serial.print("Append Item\n");
  listAppend(list, int2);
  printList(list);
  delay(1000);
  Serial.print("Delete Index\n");
  deleteIdx(list, 2);
  printList(list);
  deleteIdx(list, 0);
  printList(list);
  deleteIdx(list, 1);
  printList(list);
  deleteIdx(list, 0);
  printList(list);
}

#endif LINKED_LIST
