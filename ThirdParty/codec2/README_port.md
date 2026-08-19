# Codec2 — перенос в проект (ThirdParty)

Вокодер Codec2. Здесь лежит подмножество исходников речевого кодека и предсобранная
статическая библиотека под наш Cortex-M4F. Подключается **только в конфигурацию LAB07**
(команда `c2load` — голый замер загрузки, БЕЗ интеграции в тракт речи).

## Происхождение и лицензия

- Источник: официальный репозиторий **github.com/drowe67/codec2**, ветка `main`.
- **Зафиксированный commit: `310777b1c6f1af0bc7c72f5b32f80f6fd9136962`** (версия проекта 1.2.0).
- **Лицензия: LGPL 2.1** — полный текст в `COPYING`. Заголовки файлов Codec2 не менялись, ни один
  файл не перелицензирован. Наших правок в файлах библиотеки нет.

## Что перенесено (речевой кодек, без FreeDV/OFDM/FSK/LDPC)

`src/` — подмножество для `codec2_create/encode/decode`:
`codec2.c, sine.c, nlp.c, lpc.c, quantise.c, phase.c, interp.c, postfilter.c, lsp.c, pack.c,
mbest.c, codec2_fft.c, kiss_fft.c, kiss_fftr.c, newamp1.c, dump.c` + заголовки Codec2 +
`src/codec2/version.h` (сгенерирован из `cmake/version.h.in`, 1.2.0).

Кодовые книги (`codebook.c, codebookd.c, codebookjmv.c, codebookge.c, codebooknewamp1.c,
codebooknewamp1_energy.c`) в исходниках Codec2 **генерируются** утилитой `generate_codebook` из
данных `src/codebook/*.txt`. Генератор воспроизведён на Python — `tools/gen_codebook.py` (логика
`src/generate_codebook.c`, LGPL; значения округляются через **float32**, как хранение `float` в
эталоне). Режимы 450/newamp2 не переносились (в дереве отсутствуют).
**Побитно сверено** (см. `docs/REPORT_codec2_codebook_verify.md`): все 6 книг байт-в-байт совпадают
с выводом штатного `generate_codebook.c`, собранного хостовым gcc из commit `310777b`.
`version.h` — штатный вывод `cmake configure_file` (1.2.0).

## Как собрана `Lib/libcodec2_cm4.a`

Тем же тулчейном, что и проект (GNU Tools for STM32 12.3.1), с ABI-флагами проекта:

```
-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -std=gnu11
-O3 -fsingle-precision-constant -ffunction-sections -fdata-sections -D__EMBEDDED__ -I src
```

- `-O3 -fsingle-precision-constant` — стандартные флаги (не правка исходников). `-fsingle-precision-
  constant` убирает почти всю эмуляцию double: soft-float `__aeabi_dadd/dmul/ddiv/dsub` в объектах
  падает **с 39 до 2 ссылок** (nm) — double шёл из констант `M_PI/PI/TWO_PI` и литералов
  (объявленных `double` в коде нет). Остаток (floor/pow/round) — редкие double-libm-вызовы; убрать
  их без правки исходников можно только `-ffast-math` (меняет семантику → риск качества, не
  применялось). Меняет точность констант — качество речи проверить перед аудио-интеграцией.
- Прежняя сборка `961e9a2` была `-O2` без `-fsingle-precision-constant` (39 double-ссылок) — на ней
  снят первый замер (~2× бюджета). См. `docs/REPORT_codec2_stack_and_speed.md`.
- `-D__EMBEDDED__` — кодовые книги идут в **flash** (`static const`), а аллокатор Codec2
  становится внешним: библиотека зовёт **`codec2_malloc/codec2_calloc/codec2_free`**, которые
  предоставляет наш код (`App/Labs/speech.c`, статический пул — без кучи).
- Проверено `nm`: неразрешённые символы `.a` — только libc/libm (`malloc/memcpy/sinf/cosf/expf/
  powf/sqrtf/...`), libgcc (`__aeabi_*`) и наши `codec2_*`. Недостающих файлов Codec2 нет.
- **Важно (поправка к разведке):** при `-O2` библиотека всё же использует double-арифметику
  (soft-float `__aeabi_dadd/dmul/ddiv`) в ряде файлов (FFT, LPC, NLP, phase, interp, newamp1) —
  на одинарном FPU это эмуляция, реальная цена вскрывается замером `c2load`.

## Пересборка `.a` (при обновлении)

Сгенерировать книги через `tools/gen_codebook.py` (см. рецепты в `src/CMakeLists.txt` оригинала:
`lsp_cb←lsp1..10`, `lsp_cbd←dlsp1..10`, `lsp_cbjmv←lspjmv1..3`, `ge_cb←gecb`,
`newamp1vq_cb←train_120_1..2`, `newamp1_energy_cb←newamp1_energy_q`), затем скомпилировать набор
выше и `ar rcs`. Точный commit — см. выше.
