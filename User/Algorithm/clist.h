#ifndef LIST_H_
#define LIST_H_

#include "struct_typedef.h"

typedef struct Node
{
    void *data;
    struct Node *next;
} Node;

typedef struct
{
    Node *head; // 头指针
    // Node *tail; // 尾指针
    uint32_t len;
} List;

/*
 *    ListCreate 创建一个链表
 *
 *    ListPushBack 尾部添加数据
 *    ListPushFront 头部添加数据
 *    ListInsert 按位置插入数据
 *    ListInsertForNode 给定结点插入数据，插入到结点前
 *
 *    ListEraseBack 尾部删除
 *    ListEraseFront 头部删除
 *    ListEraseForNode 给定结点删除
 *
 *    ListGetHead 获取头结点
 *    ListGetTail 获取尾结点
 *    ListGetNode 获取指定位置的结点
 *    ListGetNodeNext 获取指定结点的下一个结点
 *
 *    ListDestroy 销毁链表
 */

extern List *ListCreate(void);

extern void ListPushBack(List *list, void *data);
extern void ListPushFront(List *list, void *data);
extern int8_t ListInsert(List *list, uint32_t pos, void *data);
extern int8_t ListInsertForNode(List *list, Node *pos, void *data);

extern void ListEraseBack(List *list);
extern void ListEraseFront(List *list);
extern void ListEraseForNode(List *list, Node *pos);

extern Node *ListGetHead(List *list);
extern Node *ListGetTail(List *list);
extern Node *ListGetNode(List *list, uint32_t pos);
extern Node *ListGetNodeNext(Node *node);

extern void ListDestroy(List *list);
#endif /* LIST_H_ */
