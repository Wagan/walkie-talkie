/**
  ******************************************************************************
  * @file    App/Common/Src/console.c
  * @author  Wagan Sarukhanov
  * @brief   Консоль поверх USB CDC (реализация). См. console.h.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Компилируется только там, где нужна консоль — обёрнут в `#if LAB_ID == 5`. Для LAB00/
  * LAB02 файл пуст (их поведение не меняется; слабый Trace_UsbPutChar остаётся пустым).
  *
  * Два кольца без блокировок:
  *  - RX: пишется из колбэка USB (ISR) Console_RxFromISR, читается в Console_Process;
  *  - TX: пишется в основном цикле (эхо/редактор, Console_Printf/Write, зеркало printf),
  *    выталкивается порциями в USB в Console_Process (неблокирующе, с учётом USBD_BUSY).
  *
  * СТРОЧНЫЙ РЕДАКТОР (Console_Process):
  *  - эхо: печатаемые символы отражаются обратно (локальное эхо в терминале не нужно);
  *  - конец строки: CR, LF или CRLF считаются ОДНИМ окончанием (LF после CR проглатывается);
  *    пустая строка не вызывает разбор и не даёт ошибку;
  *  - забой: Backspace (0x08) и DEL (0x7F) стирают последний символ; Ctrl-U (0x15) — всю строку;
  *  - история: до CONSOLE_HIST_SIZE команд, листается стрелками вверх/вниз (ESC[A / ESC[B),
  *    escape-последовательности разбираются автоматом и не попадают в строку;
  *  - приглашение "> " печатается, когда консоль ждёт ввод.
  *
  * АСИНХРОННЫЙ ВЫВОД БЕЗ ПОТЕРЬ ВВОДА: любой вывод сообщения (Console_Printf/Write и зеркало
  * printf) перед печатью стирает с экрана текущую строку ввода (msg_pre), а в конце
  * Console_Process строка ввода перерисовывается (приглашение + набранное). Набранный текст
  * хранится в буфере line[] и на экране восстанавливается — он не теряется.
  ******************************************************************************
  */

#include "lab.h"

#if (LAB_ID == 3) || (LAB_ID == 4) || (LAB_ID == 5) || (LAB_ID == 7)

#include "console.h"
#include "usbd_cdc_if.h"   /* CDC_Transmit_FS + USBD_* типы/состояния (через usbd_cdc.h) */
#include "stm32f4xx_hal.h" /* HAL_Delay для Console_Flush */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* Дескриптор USB Device (определён в USB_DEVICE/App/usb_device.c). */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* ================= Параметры ================= */
#define RX_RING     256u
#define TX_RING     2048u
#define TX_CHUNK    64u                 /* размер порции в USB (пакет FS = 64 байта) */
#define LINE_MAX    CONSOLE_LINE_MAX
#define HIST_SIZE   CONSOLE_HIST_SIZE
#define MAX_ARGS    8
#define PROMPT      "> "
#define PROMPT_LEN  2u

/* ================= Приёмное кольцо (ISR -> main) ================= */
static uint8_t           rxRing[RX_RING];
static volatile uint16_t rxHead = 0u;
static uint16_t          rxTail = 0u;

/* ================= Передающее кольцо (main -> USB) ================= */
static uint8_t  txRing[TX_RING];
static uint16_t txHead = 0u;
static uint16_t txTail = 0u;
static uint8_t  txChunk[TX_CHUNK];

/* ================= Редактор строки ================= */
static char     line[LINE_MAX];
static uint16_t len       = 0u;         /* длина набранной строки */
static uint8_t  inputShown = 0u;        /* нарисованы ли на экране приглашение+строка */
static uint8_t  skipLF    = 0u;         /* проглотить LF, идущий сразу за CR */

/* Разбор escape-последовательностей стрелок: NONE -> ESC(0x1B) -> CSI('[') -> финал. */
enum { ESC_NONE = 0, ESC_GOT_ESC, ESC_GOT_CSI };
static uint8_t  escState = ESC_NONE;
static uint16_t escNum   = 0u;          /* числовой параметр CSI (напр. 3 в ESC[3~) */

/* История команд (кольцо). */
static char     histBuf[HIST_SIZE][LINE_MAX];
static uint8_t  histCount = 0u;         /* сколько записей заполнено (<= HIST_SIZE) */
static uint8_t  histHead  = 0u;         /* индекс следующей записи */
static uint8_t  histPos   = 0u;         /* 0 — не листаем; 1..histCount — глубина назад */

/* ================= Таблица команд ================= */
static const console_cmd_t *g_cmds  = NULL;
static uint16_t             g_count = 0u;

/* ================= Низкий уровень вывода (в TX-кольцо, без логики сообщений) ================= */
static void raw_put(uint8_t c)
{
  uint16_t nxt = (uint16_t)((txHead + 1u) % TX_RING);
  if (nxt == txTail) { return; }        /* буфер полон — символ теряется (консоль, не критично) */
  txRing[txHead] = c;
  txHead = nxt;
}

static void raw_write(const char *s)
{
  while (*s != '\0') { raw_put((uint8_t)*s); s++; }
}

/* Стереть с экрана текущую строку ввода (приглашение + набранное), не завися от ANSI:
 * возврат каретки, пробелы поверх всей ширины, снова возврат каретки. */
static void erase_input_visible(void)
{
  uint16_t vis = (uint16_t)(PROMPT_LEN + len);
  uint16_t i;
  raw_put((uint8_t)'\r');
  for (i = 0u; i < vis; i++) { raw_put((uint8_t)' '); }
  raw_put((uint8_t)'\r');
}

/* Перерисовать приглашение и набранную строку. */
static void redraw_input(void)
{
  uint16_t i;
  raw_write(PROMPT);
  for (i = 0u; i < len; i++) { raw_put((uint8_t)line[i]); }
  inputShown = 1u;
}

/* Перед печатью СООБЩЕНИЯ: если на экране строка ввода — стереть её (один раз). */
static void msg_pre(void)
{
  if (inputShown != 0u)
  {
    erase_input_visible();
    inputShown = 0u;
  }
}

/* ================= Публичный вывод сообщений ================= */
void Console_Write(const char *s)
{
  if (s == NULL) { return; }
  msg_pre();
  raw_write(s);
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
  msg_pre();
  for (i = 0; i < n; i++) { raw_put((uint8_t)tmp[i]); }
  return n;
}

/* Зеркало printf в USB: сильное переопределение слабого хука из trace_swo.c (TRACE_LOG/ERR
 * попадают И в SWO, И в USB — только в LAB05). Тоже сообщение — проходит через msg_pre. */
void Trace_UsbPutChar(int ch)
{
  msg_pre();
  raw_put((uint8_t)ch);
}

/* Вытолкнуть одну порцию из TX-кольца в USB (неблокирующе). */
static void tx_drain(void)
{
  uint16_t avail;
  uint16_t n;
  uint16_t i;

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) { return; }

  avail = (uint16_t)((txHead + TX_RING - txTail) % TX_RING);
  if (avail == 0u) { return; }
  n = (avail < TX_CHUNK) ? avail : TX_CHUNK;

  for (i = 0u; i < n; i++)               /* peek: не сдвигаем tail до успеха */
  {
    txChunk[i] = txRing[(uint16_t)((txTail + i) % TX_RING)];
  }
  if (CDC_Transmit_FS(txChunk, n) == USBD_OK)
  {
    txTail = (uint16_t)((txTail + n) % TX_RING);
  }
}

void Console_Flush(void)
{
  uint32_t guard;
  for (guard = 0u; guard < 300u; guard++)          /* не дольше ~300 мс */
  {
    if (((uint16_t)((txHead + TX_RING - txTail) % TX_RING)) == 0u) { break; }
    tx_drain();
    HAL_Delay(1u);
  }
}

/* ================= Приём (ISR) ================= */
void Console_RxFromISR(const uint8_t *data, uint32_t len_in)
{
  uint32_t i;
  uint16_t nxt;

  if (data == NULL) { return; }
  for (i = 0u; i < len_in; i++)
  {
    nxt = (uint16_t)((rxHead + 1u) % RX_RING);
    if (nxt == rxTail) { return; }       /* переполнение — остаток теряется */
    rxRing[rxHead] = data[i];
    rxHead = nxt;
  }
}

/* ================= История ================= */
static const char *hist_at(uint8_t depth)   /* depth: 1 — новейшая, .. histCount — старейшая */
{
  uint8_t idx = (uint8_t)((histHead + HIST_SIZE - depth) % HIST_SIZE);
  return histBuf[idx];
}

static void hist_push(const char *s)
{
  if (s[0] == '\0') { return; }
  if (histCount > 0u)
  {
    if (strcmp(hist_at(1u), s) == 0) { return; }   /* не дублируем подряд */
  }
  strncpy(histBuf[histHead], s, LINE_MAX - 1u);
  histBuf[histHead][LINE_MAX - 1u] = '\0';
  histHead = (uint8_t)((histHead + 1u) % HIST_SIZE);
  if (histCount < HIST_SIZE) { histCount++; }
}

/* Заменить видимую строку ввода на s (стереть набранное посимвольно, напечатать новое). */
static void replace_line(const char *s)
{
  uint16_t i = 0u;
  while (len > 0u) { raw_write("\b \b"); len--; }
  while ((s[i] != '\0') && (i < (LINE_MAX - 1u)))
  {
    line[i] = s[i];
    raw_put((uint8_t)s[i]);
    i++;
  }
  len = i;
  line[len] = '\0';
}

static void hist_up(void)
{
  if (histPos < histCount) { histPos++; replace_line(hist_at(histPos)); }
}

static void hist_down(void)
{
  if (histPos > 0u)
  {
    histPos--;
    replace_line((histPos == 0u) ? "" : hist_at(histPos));
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
static void dispatch(char *cmdline)
{
  char *argv[MAX_ARGS];
  int   argc = 0;
  char *s = cmdline;
  uint16_t i;

  while (*s != '\0')
  {
    while ((*s == ' ') || (*s == '\t')) { s++; }
    if (*s == '\0') { break; }
    if (argc < MAX_ARGS) { argv[argc++] = s; }
    while ((*s != '\0') && (*s != ' ') && (*s != '\t')) { s++; }
    if (*s != '\0') { *s++ = '\0'; }
  }
  if (argc == 0) { return; }            /* только пробелы — не ошибка */

  if (strcmp(argv[0], "help") == 0) { builtin_help(); return; }
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

/* Завершение строки (Enter). Пустая строка — молча, без ошибки. */
static void submit_line(void)
{
  raw_write("\r\n");                    /* визуально завершить строку ввода */
  inputShown = 0u;
  if (len > 0u)
  {
    line[len] = '\0';
    hist_push(line);
    dispatch(line);                     /* dispatch печатает ответ через Console_* (msg_pre) */
  }
  len = 0u;
  histPos = 0u;
}

/* ================= Обработка одного принятого байта ================= */
static void handle_byte(uint8_t c)
{
  /* 1) Разбор escape-последовательностей (стрелки и т.п.) — вне обычного ввода. */
  if (escState == ESC_GOT_ESC)
  {
    if (c == (uint8_t)'[') { escState = ESC_GOT_CSI; escNum = 0u; }
    else                   { escState = ESC_NONE; }   /* неизвестная ESC-последовательность */
    return;
  }
  if (escState == ESC_GOT_CSI)
  {
    if ((c >= (uint8_t)'0') && (c <= (uint8_t)'9'))
    {
      escNum = (uint16_t)(escNum * 10u + (uint16_t)(c - (uint8_t)'0'));
      return;
    }
    escState = ESC_NONE;
    if      (c == (uint8_t)'A') { hist_up(); }        /* стрелка вверх */
    else if (c == (uint8_t)'B') { hist_down(); }      /* стрелка вниз  */
    else if ((c == (uint8_t)'~') && (escNum == 3u))   /* клавиша Delete -> забой */
    {
      if (len > 0u) { raw_write("\b \b"); len--; histPos = 0u; }
    }
    /* 'C'/'D' (вправо/влево) и прочее — игнорируем (курсор только в конце) */
    return;
  }
  if (c == 0x1Bu) { escState = ESC_GOT_ESC; return; }

  /* 2) Окончания строк: CR, LF, CRLF -> одно окончание. */
  if (c == (uint8_t)'\r') { skipLF = 1u; submit_line(); return; }
  if (c == (uint8_t)'\n')
  {
    if (skipLF != 0u) { skipLF = 0u; return; }        /* LF сразу за CR — проглотить */
    submit_line();
    return;
  }
  skipLF = 0u;

  /* 3) Редактирование. */
  if ((c == 0x08u) || (c == 0x7Fu))     /* Backspace / DEL — забой */
  {
    if (len > 0u) { raw_write("\b \b"); len--; histPos = 0u; }
    return;
  }
  if (c == 0x15u)                        /* Ctrl-U — очистить всю строку */
  {
    while (len > 0u) { raw_write("\b \b"); len--; }
    histPos = 0u;
    return;
  }

  /* 4) Печатаемый символ — в строку + эхо. */
  if ((c >= 0x20u) && (c <= 0x7Eu))
  {
    if (len < (LINE_MAX - 1u))
    {
      line[len++] = (char)c;
      raw_put(c);                        /* эхо */
      histPos = 0u;
    }
  }
  /* прочие управляющие символы — игнорируем */
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
  len = 0u; inputShown = 0u; skipLF = 0u;
  escState = ESC_NONE; escNum = 0u;
  histCount = 0u; histHead = 0u; histPos = 0u;
  line[0] = '\0';
}

void Console_Process(void)
{
  /* 1) Разобрать всё, что накопилось в приёмном кольце. Перед эхом убедиться, что строка
   *    ввода нарисована (иначе эхо ушло бы без приглашения). */
  while (rxTail != rxHead)
  {
    uint8_t c = rxRing[rxTail];
    rxTail = (uint16_t)((rxTail + 1u) % RX_RING);
    if (inputShown == 0u) { redraw_input(); }
    handle_byte(c);
  }

  /* 2) В покое строка ввода должна быть на экране (приглашение + набранное). */
  if (inputShown == 0u) { redraw_input(); }

  /* 3) Вытолкнуть порцию вывода в USB (неблокирующе). */
  tx_drain();
}

#endif /* LAB_ID == 5 */
