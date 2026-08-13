/**
  ******************************************************************************
  * @file    App/Common/Src/console.c
  * @author  Wagan Sarukhanov
  * @brief   Консоль поверх USB CDC (реализация). См. console.h.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Компилируется только там, где нужна консоль — обёрнут в `#if LAB_ID == 5`. Для LAB00/
  * LAB02 файл пуст, поэтому их поведение не меняется (в частности, слабый Trace_UsbPutChar
  * из trace_swo.c остаётся пустым, и TRACE_LOG у них идёт только в SWO).
  *
  * Два кольца без блокировок:
  *  - RX: пишется из колбэка USB (ISR) Console_RxFromISR, читается в Console_Process;
  *  - TX: пишется в основном цикле (Console_Printf/Write и зеркало printf Trace_UsbPutChar),
  *    выталкивается порциями в USB в Console_Process (неблокирующе, с учётом USBD_BUSY).
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 5

#include "console.h"
#include "usbd_cdc_if.h"   /* CDC_Transmit_FS + USBD_* типы/состояния (через usbd_cdc.h) */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* Дескриптор USB Device (определён в USB_DEVICE/App/usb_device.c). */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* ================= Параметры ================= */
#define RX_RING     256u
#define TX_RING     2048u
#define TX_CHUNK    64u      /* размер порции в USB (пакет FS = 64 байта) */
#define LINE_MAX    128u
#define MAX_ARGS    8

/* ================= Приёмное кольцо (ISR -> main) ================= */
static uint8_t           rxRing[RX_RING];
static volatile uint16_t rxHead = 0u;
static uint16_t          rxTail = 0u;

/* ================= Передающее кольцо (main -> USB) ================= */
static uint8_t  txRing[TX_RING];
static uint16_t txHead = 0u;
static uint16_t txTail = 0u;
static uint8_t  txChunk[TX_CHUNK];

/* ================= Сборка строки и таблица команд ================= */
static char     lineBuf[LINE_MAX];
static uint16_t lineLen = 0u;

static const console_cmd_t *g_cmds  = NULL;
static uint16_t             g_count = 0u;

/* ================= Вывод ================= */
static void tx_put(uint8_t c)
{
  uint16_t nxt = (uint16_t)((txHead + 1u) % TX_RING);
  if (nxt == txTail)
  {
    return;                    /* буфер полон — символ теряется (консоль, не критично) */
  }
  txRing[txHead] = c;
  txHead = nxt;
}

void Console_Write(const char *s)
{
  if (s == NULL) { return; }
  while (*s != '\0')
  {
    tx_put((uint8_t)*s);
    s++;
  }
}

int Console_Printf(const char *fmt, ...)
{
  char    tmp[256];
  int     n;
  int     i;
  va_list ap;

  va_start(ap, fmt);
  n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);

  if (n < 0) { return n; }
  if (n > (int)(sizeof(tmp) - 1u)) { n = (int)(sizeof(tmp) - 1u); }  /* усечение */
  for (i = 0; i < n; i++)
  {
    tx_put((uint8_t)tmp[i]);
  }
  return n;
}

/* Зеркало printf в USB: сильное переопределение слабого хука из trace_swo.c. Благодаря ему
 * TRACE_LOG/TRACE_ERR (они идут через printf) попадают И в SWO, И в USB — но только в
 * конфигурации с консолью (LAB05), т.к. этот файл компилируется лишь при LAB_ID==5. */
void Trace_UsbPutChar(int ch)
{
  tx_put((uint8_t)ch);
}

/* Вытолкнуть одну порцию из TX-кольца в USB (неблокирующе). */
static void tx_drain(void)
{
  uint16_t avail;
  uint16_t n;
  uint16_t i;

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    return;                    /* хост ещё не сконфигурировал устройство */
  }

  avail = (uint16_t)((txHead + TX_RING - txTail) % TX_RING);
  if (avail == 0u) { return; }
  n = (avail < TX_CHUNK) ? avail : TX_CHUNK;

  /* Peek: копируем, НЕ сдвигая tail, чтобы при USBD_BUSY данные не потерялись. */
  for (i = 0u; i < n; i++)
  {
    txChunk[i] = txRing[(uint16_t)((txTail + i) % TX_RING)];
  }
  if (CDC_Transmit_FS(txChunk, n) == USBD_OK)
  {
    txTail = (uint16_t)((txTail + n) % TX_RING);   /* отдано — можно сдвинуть */
  }
}

/* ================= Приём (ISR) ================= */
void Console_RxFromISR(const uint8_t *data, uint32_t len)
{
  uint32_t i;
  uint16_t nxt;

  if (data == NULL) { return; }
  for (i = 0u; i < len; i++)
  {
    nxt = (uint16_t)((rxHead + 1u) % RX_RING);
    if (nxt == rxTail)
    {
      return;                  /* переполнение — остаток теряется */
    }
    rxRing[rxHead] = data[i];
    rxHead = nxt;
  }
}

/* ================= Встроенная команда help ================= */
static void builtin_help(void)
{
  uint16_t i;
  Console_Write("commands:\r\n");
  Console_Write("  help          - list commands\r\n");
  for (i = 0u; i < g_count; i++)
  {
    Console_Printf("  %-12s- %s\r\n",
                   (g_cmds[i].name != NULL) ? g_cmds[i].name : "?",
                   (g_cmds[i].help != NULL) ? g_cmds[i].help : "");
  }
}

/* ================= Разбор и диспетчеризация строки ================= */
static void dispatch(char *line)
{
  char *argv[MAX_ARGS];
  int   argc = 0;
  char *s = line;
  uint16_t i;

  /* Токенизация по пробелам/табам (правка строки на месте). */
  while (*s != '\0')
  {
    while ((*s == ' ') || (*s == '\t')) { s++; }
    if (*s == '\0') { break; }
    if (argc < MAX_ARGS) { argv[argc++] = s; }
    while ((*s != '\0') && (*s != ' ') && (*s != '\t')) { s++; }
    if (*s != '\0') { *s++ = '\0'; }
  }
  if (argc == 0) { return; }

  if (strcmp(argv[0], "help") == 0)
  {
    builtin_help();
    return;
  }
  for (i = 0u; i < g_count; i++)
  {
    if ((g_cmds[i].name != NULL) && (strcmp(argv[0], g_cmds[i].name) == 0))
    {
      if (g_cmds[i].fn != NULL) { g_cmds[i].fn(argc, argv); }
      return;
    }
  }
  Console_Printf("unknown command: %s (try 'help')\r\n", argv[0]);
}

/* ================= Интерфейс ================= */
void Console_Register(const console_cmd_t *cmds, uint16_t count)
{
  g_cmds  = cmds;
  g_count = count;
}

uint8_t Console_IsConfigured(void)
{
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1u : 0u;
}

void Console_Init(void)
{
  rxHead = 0u; rxTail = 0u;
  txHead = 0u; txTail = 0u;
  lineLen = 0u;
}

void Console_Process(void)
{
  /* 1) Разобрать всё, что накопилось в приёмном кольце, собирая строки. Терминалы шлют
   *    разные окончания (CR, LF, CRLF) — считаем строкой всё до любого из них; пустые
   *    строки (второй символ CRLF) игнорируются. Backspace/DEL стирают последний символ. */
  while (rxTail != rxHead)
  {
    uint8_t c = rxRing[rxTail];
    rxTail = (uint16_t)((rxTail + 1u) % RX_RING);

    if ((c == (uint8_t)'\r') || (c == (uint8_t)'\n'))
    {
      if (lineLen > 0u)
      {
        lineBuf[lineLen] = '\0';
        dispatch(lineBuf);
        lineLen = 0u;
      }
    }
    else if ((c == 0x08u) || (c == 0x7Fu))
    {
      if (lineLen > 0u) { lineLen--; }
    }
    else if (lineLen < (LINE_MAX - 1u))
    {
      lineBuf[lineLen++] = (char)c;
    }
    /* иначе — переполнение строки: символ отбрасываем */
  }

  /* 2) Вытолкнуть порцию вывода в USB (неблокирующе). */
  tx_drain();
}

#endif /* LAB_ID == 5 */
