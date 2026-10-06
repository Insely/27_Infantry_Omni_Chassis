#include "clist.h"

#include <stdlib.h>

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

/// @brief 创建一个链表
List * ListCreate(void)
{
    List * list = (List *)malloc(sizeof(List));
    if (list == NULL) {
        return NULL;
    }
    list->head = NULL;
    list->len = 0;
    return list;
}

/// @brief 尾部添加数据
void ListPushBack(List * list, void * data)
{
    if (list == NULL) {
        return;
    }

    Node * node = (Node *)malloc(sizeof(Node));
    if (node == NULL) {
        return;
    }
    node->data = data;
    node->next = NULL;
    if (list->head == NULL) {
        list->head = node;
    } else {
        Node * p = list->head;
        while (p->next != NULL) {
            p = p->next;
        }
        p->next = node;
    }
    list->len++;
}

/// @brief 头部添加数据
void ListPushFront(List * list, void * data)
{
    if (list == NULL) {
        return;
    }

    Node * node = (Node *)malloc(sizeof(Node));
    if (node == NULL) {
        return;
    }
    node->data = data;
    node->next = list->head;
    list->head = node;
    list->len++;
}

/// @brief 按位置插入数据
int8_t ListInsert(List * list, uint32_t pos, void * data)
{
    if (list == NULL || pos > list->len + 1) {
        return -1;
    }

    if (pos == 0) {
        ListPushFront(list, data);
        return 0;
    }

    Node * node = (Node *)malloc(sizeof(Node));
    if (node == NULL) {
        return -1;
    }
    node->data = data;
    node->next = NULL;

    Node * p = list->head;
    for (int i = 0; i < pos - 1; i++) {
        p = p->next;
    }
    node->next = p->next;
    p->next = node;
    list->len++;
    return 0;
}

/// @brief 给定结点插入数据，插入到结点前
int8_t ListInsertForNode(List * list, Node * pos, void * data)
{
    if (list == NULL || pos == NULL) {
        return -1;
    }

    if (list->head == pos) {
        ListPushFront(list, data);
        return 0;
    }

    Node * node = (Node *)malloc(sizeof(Node));
    if (node == NULL) {
        return -1;
    }

    Node * p = list->head;
    uint32_t cnt = 0;
    while (p->next != pos) {
        p = p->next;
        cnt++;
    }
    if (cnt == list->len) {
        return -1;
    }
    node->data = data;
    node->next = pos;

    p->next = node;
    list->len++;
    return 0;
}

/// @brief 尾部删除
void ListEraseBack(List * list)
{
    if (list == NULL || list->head == NULL) {
        return;
    }

    if (list->head->next == NULL) {
        free(list->head);
        list->head = NULL;
        list->len--;
        return;
    }

    Node * p = list->head;
    Node * q = p->next;

    while (q->next != NULL) {
        p = q;
        q = q->next;
    }
    free(q);
    p->next = NULL;
    list->len--;
}

/// @brief 头部删除
void ListEraseFront(List * list)
{
    if (list == NULL || list->head == NULL) {
        return;
    }

    Node * p = list->head;
    list->head = p->next;
    free(p);
    list->len--;
}

/// @brief 给定结点删除
void ListEraseForNode(List * list, Node * pos)
{
    if (list == NULL || pos == NULL) {
        return;
    }

    if (list->head == pos) {
        ListEraseFront(list);
        return;
    }

    Node * p = list->head;
    uint32_t cnt = 0;
    while (p->next != pos) {
        p = p->next;
        cnt++;
    }

    if (cnt == list->len) {
        return;
    }

    p->next = pos->next;
    free(pos);
    list->len--;
}

/// @brief 获取头结点
Node * ListGetHead(List * list)
{
    if (list == NULL) {
        return NULL;
    }
    return list->head;
}

/// @brief 获取尾结点
Node * ListGetTail(List * list)
{
    if (list == NULL || list->head == NULL) {
        return NULL;
    }

    Node * p = list->head;
    while (p->next != NULL) {
        p = p->next;
    }
    return p;
}

/// @brief 获取指定位置的结点
Node * ListGetNode(List * list, uint32_t pos)
{
    if (list == NULL || pos >= list->len) {
        return NULL;
    }

    Node * p = list->head;
    for (int i = 0; i < pos; i++) {
        p = p->next;
    }
    return p;
}

/// @brief 获取指定结点的下一个结点
Node * ListGetNodeNext(Node * node)
{
    if (node == NULL) {
        return NULL;
    }
    return node->next;
}

/// @brief 销毁链表
void ListDestroy(List * list)
{
    if (list == NULL) {
        return;
    }

    Node * p = list->head;
    Node * q = NULL;
    while (p != NULL) {
        q = p->next;
        free(p);
        p = q;
    }
    free(list);
}
