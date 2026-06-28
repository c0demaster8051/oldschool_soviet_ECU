; настройка синхронизации по шкиву
sync_start_mult equ 30h
sync_half_num equ 31h
sync_count_num equ 32h

; основные переменные
ign_num equ 33h
ign_offset equ 34h
ign_ctrl equ 35h
fuel_num equ 36h
fuel_lo equ 37h
fuel_hi equ 38h
adc_num equ 39h

; накопленный период оборота скидывается сюда
period_lo equ 3Ah
period_mi equ 3Bh
period_hi equ 3Ch

; в процессе обмена по шине обновляются временные значения,
; которые потом в прерывании заменят собой постоянные
; в начале следующей половины оборота
ign_num_temp equ 3Dh
ign_offset_temp equ 3Eh
fuel_num_temp equ 3Fh

; временное хранение состояния GPIO
; старший бит меняется прерыванием
gpio_reg equ 40h

; настройки отсечки
redline_lo equ 41h
redline_hi equ 42h
redline_preset equ 43h
redline_fuel_count equ 44h
redline_ign_count equ 45h




; адрес регистра флагов не менять! он связан с перечисленными ниже
; прямоадресуемыми битами
flag_reg equ 20h
rev_half_flag equ 00h ; в какой половине оборота относительно пропуска находится шкив
phase_allow_flag equ 01h ; разрешение перехода в фазированный впрыск
redline_fuel_flag equ 02h ; флаг отсечки
redline_ign_flag equ 03h ; флаг отсечки
; следующие 4 флага доступны для чтения командой статус
sync_reach_flag equ 04h ; признак вращения движка и достижения синхронизации
rev_parity_flag equ 05h ; чётный/нечётный оборот для фазированного впрыска
sync_err_flag equ 06h ; ошибка синхронизации, если число зубов до пропуска не совпало
comm_err_flag equ 07h ; неверная команда или контрольная сумма при обмене данными






org 0h
ajmp init


org 03h ; Внешнее прерывание INT0
; Входы INT0 и INT1 подключены к комплементарным выходам триггера
; схемы синхронизации по ДПКВ, поэтому их состояние всегда противоположно.
; прерывание вызывается падающим фронтом на выводе INT0,
; низкий уровень на нём останавливает таймер 0, и запускает таймер 1
; мы можем считать период из остановленного и сбросить его для нового отсчёта
ajmp tim_0_rdy_int


org 0bh ;Пpepывaниe oт тaймepa 0
; переполнение любого из таймеров обозначает остановку двигателя
; и срыв синхронизации, после чего при повторной прокрутке стартером
; надо снова ловить пропуск зубов на шкиву. сбрасываем бит синхронизации
; останавливаем таймер и пишем в него максимальное значение
clr TR0
mov TH0,#0FFh
sjmp tim_ovf_end


org 13h ; Внешнее прерывание INT1
; аналогично прерыванию INT0, только здесь обслуживаем таймер 1
ajmp tim_1_rdy_int


org 1bh ; Прерывание от таймера 1
; аналогично таймеру 0
clr TR1
mov TH1,#0FFh
sjmp tim_ovf_end


org 23h ; прерывание от UART
; Не используется
reti


org 24h
tim_ovf_end:
clr sync_reach_flag ; сбрасываем флаг синхронизации
; и обновляем данные
mov ign_num,ign_num_temp
mov ign_offset,ign_offset_temp
mov fuel_num,fuel_num_temp
reti


org 40h
; R0 - полный номер зуба
; R1 - "симметричный" номер зуба
; R2, R3, R4 - накопитель периода оборота
; R5 - общая переменная для расчёта
; R6, R7 - предыдущее время между зубами шкива
tim_1_rdy_int:
push PSW
push ACC
push B
push DPL
; выбираем первый банк регистров
mov PSW,#08h
; прибавляем новое значение к накопителю
; на последнем зубе в накопителе будет полный период оборота двигателя
mov A,R2
add A,TL1
mov R2,A
mov A,R3
addc A,TH1
mov R3,A
clr A
addc A,R4
mov R4,A
; теперь сравниваем свежее значение с предыдущим для поиска пропущенных зубов
; для этого свежее значение умножаем на  уменьшающий коэффициент
mov A,TL1
mov B,sync_start_mult
mul AB
mov R5,B
mov A,TH1
mov B,sync_start_mult
mul AB
add A,R5
cpl A
mov R5,A ; теперь в R5 инвертированный младший байт
clr A
addc A,B
cpl A
xch A,0Dh ; xch R5
; а теперь в R5 инвертированный старший байт
; а в аккумуляторе инвертированный младший
; складываем с младшим предыдущего результата
add A,R6 ; сумма нас не интересует, нужен только перенос
mov A,R5 ; складываем старший инвертированный
; со старшим предыдущего измерения
addc A,R7 ; и если нет флага переноса - значит текущее измерение
; даже с учётом понижающего коэффициента оказалось больше предыдущего результата
; а это значит что только что мимо датчика проскочил пропуск на шкиву
; и этот результат мы не будем сохранять в качестве предыдущего
jnc tim_1_clr
; сохраняем значение таймера
mov R6,TL1
mov R7,TH1
tim_1_clr:
mov TL1,#0
mov TH1,#0
setb TR1 ; запускаем таймер, если двигатель был остановлен
sjmp sync_branch
; далее абсолютно аналогичный кусок для второго таймера
tim_0_rdy_int:
push PSW
push ACC
push B
push DPL
mov PSW,#08h
mov A,R2
add A,TL0
mov R2,A
mov A,R3
addc A,TH0
mov R3,A
clr A
addc A,R4
mov R4,A
mov A,TL0
mov B,sync_start_mult
mul AB
mov R5,B
mov A,TH0
mov B,sync_start_mult
mul AB
add A,R5
cpl A
mov R5,A
clr A
addc A,B
cpl A
xch A,0Dh
add A,R6
mov A,R5
addc A,R7
jnc tim_0_clr
mov R6,TL0
mov R7,TH0
tim_0_clr:
mov TL0,#0
mov TH0,#0
setb TR0 ; запускаем таймер, если двигатель был остановлен
sync_branch:
mov A,R0
jc no_sync ; если обнаружили пропуск
cpl rev_parity_flag ; инвертируем бит номера оборота для фазированного режима
; ставим флаг синхронизации
setb sync_reach_flag
; номер зуба - 0
mov R0,#0h
; если номер зуба не равен 0 - сбой синхронизации, прыгаем на его обработчик
cjne A,#0,sync_fail
sjmp no_sync
sync_fail:
; вероятно поймали короткую импульсную помеху, и сохранённый период между импульсами
; очень короткий, пишем максимальный период между импульсами
mov R6,#0FFh
mov R7,#0FFh
setb sync_err_flag ; ставим флаг ошибки синхронизации
clr A
no_sync:
; получаем "симметричный" номер зуба для разных групп цилиндров (1-4 и 2-3)
; находим остаток от деления полного номера зуба на половину зубов шкива (30)
mov B,sync_half_num
div AB
mov C,A.0
mov rev_half_flag,C
mov A,B
mov R1,A
; теперь в аккумуляторе "симметричный" номер, а в флаге rev_half_flag
; указатель на текущую "половину" (0 - первая половина оборота, 1 - вторая)
jnz ignition_test ; в начале половины оборота происходит обновление переменных
mov ign_num,ign_num_temp
mov ign_offset,ign_offset_temp
mov fuel_num,fuel_num_temp
ignition_test:
; проверяем на какой зуб назначено зажигание, самый приоритетный процесс
cjne A,ign_num,fuel_test ; если тут...
jb redline_ign_flag,fuel_test ; если не ограничены оборотами...
jnb sync_reach_flag,fuel_test ; и если есть синхронизация...
; вычисляем смещение для таймера из актуального периода между зубами шкива
; в процессе умножения к результату прибавится единица, чтобы таймер мог правильно
; отработать смещение равное 0 (если записать в ВИ53 0 - он будет считать 65536 тактов)
; при смещении равном 0 таймер отсчитает 1 такт и выдаст искру
mov B,ign_offset
mov A,R6
mul AB
mov R5,B
mov B,ign_offset
mov A,R7
mul AB
setb C ; +1
addc A,R5
mov R5,A
clr A
addc A,B
xch A,0Dh
; теперь в аккумуляторе младший байт
; а в регистре R5 - старший
; пишем смещение в таймер
mov DPL,#04h
movx @DPTR,A
mov A,R5
movx @DPTR,A
; запись по этому адресу взводит схему разрешения счёта, которая
; в свою очередь запускает таймер на ближайшем импульсе с ДПКВ
; в конце отсчёта СРАЗУ выдаётся искра и запускается таймер на рассыпухе,
; отсчитывающий длительность открытого состояния ключа для поддержания
; многоискрового режима работы CDI
; *** далее вычисляем маску активации каналов зажигания ***
; каналы от младшего бита к старшему активируются последовательно, таким образом
; бит 0 - первая половина первого оборота
; бит 1 - вторая половина первого оборота
; бит 2 - первая половина второго оборота
; бит 3 - вторая половина второго оборота
; каналы от младшего бита к старшему для ЗМЗ соответствуют цилиндрам 1-2-4-3
mov A,#0FAh ; с самого начала выбраны каналы 0 и 2 (1 и 4 цилиндры)
jnb rev_half_flag,ign_mask_noinv ; проверяем, в какой половине оборота мы находимся
xrl A,#0Fh ; во второй половине оборота работают цилиндры 2 и 3, инвертируем маску
ign_mask_noinv:
jnb phase_allow_flag,set_ign_mask ; проверяем признак фазированного впрыска
rr A ; если оборот соответствует 1 или 2 цилиндру, сдвигаем маску вправо,
rr A ; после этого уехавшая часть маски для 3 и 4 цилиндров в старшей тетраде
jnb rev_parity_flag,set_ign_mask ; если это рабочий оборот для 3 и 4 цилиндра
swap A ; меняем тетрады местами
set_ign_mask:
; теперь в младшей половине аккумулятора сформирована маска
anl A,#0Fh ; очищаем старшую тетраду
; накладываем на итоговое значение коэффициент предделителя
orl A,ign_ctrl ; и маску блокировки каналов зажигания
mov P1,A ; пишем в порт
mov A,R1
fuel_test:
cjne A,fuel_num,adc_hold_test ; пора лить топливо? проверяем, не запрещено ли отсечкой
jb redline_fuel_flag,adc_hold_test ; и начинаем хитрую магию с битами
; в каждой из двух микросхем таймеров каналы 1 и 2 отведены под индивидуальные форсунки
; канал 1 верхний таймер - первая половина первого оборота
; канал 2 верхний таймер - вторая половина первого оборота
; канал 1 нижний таймер - первая половина второго оборота
; канал 2 нижний таймер - вторая половина второго оборота
; при этом младшие биты адреса при записи в канал 1 будут 01b, а для канала 2 - 10b
; сигналы chip_select идут прямо на адресные линии, таким образом есть возможность писать время
; открытия форсунки одновременно в 2 таймера одним действием, для нефазированного режима
; формируем адрес, куда будем писать время открытия форсунки:
mov A,#01h ; изначально выбран адрес канала 1 микросхемы
jnb rev_half_flag,chn_addr_sel ; вторая половина оборота?
rl A ; простым сдвигом выбираем второй канал таймера
chn_addr_sel:
; сформированный адрес подходит для нефазированного режима, т.к.
; запись будет выполнена сразу в оба канала таймера одновременно
jnb phase_allow_flag,fuel_chn_write
; в фазированном режиме один из каналов надо отключить установкой бита chip_select в адресе
mov C,rev_parity_flag
mov ACC.3,C
cpl C
mov ACC.2,C
fuel_chn_write:
; итоговый адрес получен, пишем время впрыска
mov DPL,A
mov A,fuel_lo
movx @DPTR,A
mov A,fuel_hi
movx @DPTR,A
mov A,R1
adc_hold_test:
; момент фиксации сигнала ДАД схемой sample&hold
cjne A,adc_num,int_end
mov A,gpio_reg
cpl ACC.7 ; инвертируем старший бит
mov gpio_reg,A
mov DPL,#0Ch
movx @DPTR,A
int_end:
; увеличиваем номер зуба 
inc R0
mov A,R0 ; сравниваем с количеством зубов шкива
cjne A,sync_count_num,tim_int_end
; оказалось что это последний зуб, так что процессорного времени
; до следующего прерывания более чем достаточно
; на последнем зубе ДПКВ сохраняем период оборота
mov period_lo,R2
mov period_mi,R3
mov period_hi,R4
clr A
mov R0,A ; обнуляем счётчик зубов
mov R2,A ; и накопитель периода оборота
mov R3,A
mov R4,A
; проверяем период для ограничения оборотов
mov A,period_hi
jnz redline_fuel_check ; старший байт не 0? обороты слишком низкие
clr C ; далее вычитаем из периода оборота значение ограничителя
mov A,period_lo
subb A,redline_lo
mov A,period_mi
subb A,redline_hi
jnc redline_fuel_check ; если в конце будет заём (carry), значит период меньше
; значения счётчиков по топливу и зажиганию в одном байте в разных тетрадах
mov A,redline_preset
mov R1,#redline_ign_count
xchd A,@R1 ; младшая тетрада уходит в счётчик зажигания
swap A
mov R1,#redline_fuel_count
xchd A,@R1 ; старшая в счётчик топлива
setb redline_ign_flag ; и сразу отключаем зажигание
redline_fuel_check:
mov A,#0FFh
add A,redline_fuel_count
mov redline_fuel_flag,C
jnc redline_ign_check
mov redline_fuel_count,A
sjmp tim_int_end
redline_ign_check:
add A,redline_ign_count
mov redline_ign_flag,C
jnc tim_int_end
mov redline_ign_count,A
tim_int_end:
pop DPL
pop B
pop ACC
pop PSW
reti



init:
; сюда переходим по сбросу проца
; сначала инициализация внешних таймеров
; настройка режимов работы
mov DPL,#07h ; верхний таймер
mov A,#32h ; таймер 0
movx @DPTR,A
mov A,#70h ; таймер 1
movx @DPTR,A
mov A,#0B0h ; таймер 2
movx @DPTR,A
mov DPL,#0Bh ; нижний таймер
mov A,#30h ; таймер 0
movx @DPTR,A
mov A,#70h ; таймер 1
movx @DPTR,A
mov A,#0B0h ; таймер 2
movx @DPTR,A
mov DPL,#0Bh
; таймеры форсунок настроены в режим 0, сразу после его установки
; выходы таймеров переходят в активное состояние, и сбрасываются только
; в конце счёта, так что пишем единицу во все эти таймеры
; общий выход форсунок
mov DPL,#08h
mov A,#01h
movx @DPTR,A
mov A,#00h
movx @DPTR,A
; форсунка 1
mov DPL,#05h
mov A,#01h
movx @DPTR,A
mov A,#00h
movx @DPTR,A
; форсунка 2
mov DPL,#06h
mov A,#01h
movx @DPTR,A
mov A,#00h
movx @DPTR,A
; форсунка 3
mov DPL,#09h
mov A,#01h
movx @DPTR,A
mov A,#00h
movx @DPTR,A
; форсунка 4
mov DPL,#0Ah
mov A,#01h
movx @DPTR,A
mov A,#00h
movx @DPTR,A



; инициализация
mov PSW,#00h ; нулевой банк регистров для фоновой задачи
mov SP,#5Fh ; настраиваем стек
; таймеры в 16-бит режим с блокировкой по входу INT
mov TMOD,#99h
; внешние прерывания по падающему фронту, таймеры пока остановлены
; они будут включены прерыванием по сигналу ДПКВ
mov TCON,#05h
; разрешить внешние прерывания и от таймера,
; глобально прерывания запрещены
mov IE,#0Fh
; выбираем режим UART с фиксированной скоростью
mov SCON,#13h
mov PCON,#00h
; предыдущее время между импульсами
mov 0Eh,#0FFh
mov 0Fh,#0FFh
; отладочное
mov sync_start_mult,#170 ; порог срабатывания по длительности пропуска
mov sync_half_num,#30 ; количество половины зубов шкива
mov sync_count_num,#58 ; полное количество зубов шкива
mov ign_num,#255
mov ign_offset,#255
mov fuel_num,#255
mov ign_num_temp,#255
mov ign_offset_temp,#255
mov fuel_num_temp,#255
mov ign_ctrl,#00h ; длительность импульса и маска запрета зажигания
mov adc_num,#255 ; 15 АЦП меряет прямо перед открытием впускного клапана
mov fuel_lo,#01h
mov fuel_hi,#00h
mov flag_reg,#00h ; DEBUG
mov redline_lo,#00h
mov redline_hi,#00h
mov redline_fuel_count,#00h
mov redline_ign_count,#00h
mov redline_preset,#00h
setb EA ; инициализация закончена, включаем прерывания


; далее цикл ожидания данных в буфере
; опрашиваем в цикле сигнал MRDY, активный уровень - 0
main_loop:
jb P3.4,main_loop
; по сигналу готовности вытягиваем первый байт
clr P3.4 ; входная логика на приём
clr P3.5 ; буфер в последовательный режим
clr RI ; запускаем процесс...
rd_1_wait:
jnb RI,rd_1_wait ; ждём готовности 1 байта
mov A,SBUF
; ноль - команда статус, оптимизирована максимально, т.к исполняется чаще всего
; команда без дополнительных данных, так что остальные байты можно не читать
; единственная команда из списка, которая может писать в буфер
jz comm_status ; сразу переходим на обработчик
clr RI ; запускаем процесс...
rd_2_wait:
jnb RI,rd_2_wait
mov R6,SBUF
clr RI
rd_3_wait:
jnb RI,rd_3_wait
mov R7,SBUF
setb P3.4 ; данные получены, логика в режим приёма
; у младших команд достаточно равенства тетрад в коде команды
; в нескольких старших командах должно выполняться равенство тетрад
; в байтах данных, в итоге двумя последними байтами в буфере передаётся
; фактически один байт данных, используется для особо ответственных мест
mov B,A ; сравниваем тетрады в коде команды
swap A
xrl A,B
jnz comm_error ; не равны? кривая команда, ставим флаг ошибки и игнорим
mov A,B ; формируем адрес для ветвления по таблице переходов
mov DPTR,#comm_jump_table
anl A,#0Fh ; нужна лишь младшая тетрада
rl A ; умноженная на 2, т.к. команды переходов длиной 2 байта
jmp @A+DPTR
comm_jump_table:
; 00-33
ajmp comm_status
ajmp comm_ignition
ajmp comm_sync_fuel
ajmp comm_async_fuel
; 44-77
ajmp comm_phase_ctrl
ajmp comm_gpio_ctrl
ajmp comm_redline_period
ajmp comm_redline_ctrl
; 88-BB
ajmp comm_error
ajmp comm_error
ajmp comm_error
ajmp comm_error
; CC-FF
ajmp comm_error
ajmp comm_sync_count
ajmp comm_sync_half
ajmp comm_sync_mult
; кривые или отсутствующие в списке команды вызывают переход сюда
comm_error:
setb comm_err_flag ; ставим флаг ошибки
sjmp buf_exch_end ; и завершаем обмен
comm_status:
setb P3.4 ; команда получена, логику на передачу
; чтобы никто не обосрал нам данные в процессе их отправки
clr EA ; максимально быстро получим их актуальную копию
mov R0,period_lo 
mov R1,period_mi
mov R2,period_hi
setb EA
mov A,R2 ; проверка старшего байта периода
anl A,#0F0h ; если в старшей тетраде что-то есть?
jz status_write ; значит вылезли за 20 бит
mov R2,#0Fh ; возвращаем 0x000FFFFF
mov R1,#0FFh
mov R0,#0FFh
status_write:
mov A,flag_reg ; подтягиваем регистр флагов
anl A,#0F0h ; выделяем его старшую половину
add A,R2 ; прибавляем младшую тетраду старшего байта периода
clr TI
mov SBUF,A ; заряжаем на передачу
wrs_1_wait:
jnb TI,wrs_1_wait
mov A,R1 ; далее средний байт периода
clr TI
mov SBUF,A
wrs_2_wait:
jnb TI,wrs_2_wait
mov A,R0 ; и младший
clr TI
mov SBUF,A
wrs_3_wait:
jnb TI,wrs_3_wait
mov A,#3Fh ; сбрасываем однократно устанавливаемые флаги
anl flag_reg,A
buf_exch_end:
setb P3.5 ; завершаем обмен по шине, буфер в параллельный режим
ajmp main_loop


comm_ignition:
; зажигание, обновить номер зуба и смещение
clr EA
mov ign_num_temp,R6
mov ign_offset_temp,R7
setb EA
mov C,TR0 ; проверяем, остановлен двигатель или нет
anl C,TR1 ; по переполнению любого из таймеров
; двигатель крутится? значит данные обновятся сами,
jc buf_exch_end ; можно выходить
clr EA ; иначе - обновляем данные ещё и напрямую
mov ign_num,R6
mov ign_offset,R7
setb EA
ajmp buf_exch_end


comm_sync_fuel:
; время впрыска
clr EA
mov fuel_hi,R6
mov fuel_lo,R7
setb EA
sjmp buf_exch_end


comm_async_fuel:
; асинхронное топливо
clr EA
mov DPL,#08h
mov A,R7
movx @DPTR,A
mov A,R6
movx @DPTR,A
setb EA
ajmp buf_exch_end


comm_phase_ctrl:
mov adc_num,R6 ; обновляем фазу измерения АЦП
mov fuel_num_temp,R7 ; и фазу впрыска
mov C,TR0 ; проверяем, остановлен двигатель или нет
anl C,TR1 ; по переполнению любого из таймеров
; двигатель крутится? значит данные обновятся сами,
jc buf_exch_end ; можно выходить
mov fuel_num,R7 ; иначе - обновляем напрямую
ajmp buf_exch_end


comm_gpio_ctrl:
mov A,R6
jnb ACC.7,phase_inv_skip
cpl rev_parity_flag
phase_inv_skip:
mov C,ACC.6
mov phase_allow_flag,C
anl A,#3Fh ; обновляем GPIO
clr EA
anl gpio_reg,#0C0h
orl gpio_reg,A
mov A,gpio_reg
mov DPL,#0Ch ; дополнительно СРАЗУ
movx @DPTR,A ; обновляем выходной регистр
setb EA
mov ign_ctrl,R7 ; и регистр управления зажиганием
ajmp buf_exch_end


comm_redline_period:
clr EA
mov redline_hi,R6
mov redline_lo,R7
setb EA
ajmp buf_exch_end


comm_redline_ctrl:
mov redline_preset,R7
ajmp buf_exch_end


comm_sync_count:
mov B,R6
mov A,R7
cpl A
cjne A,B,data_check_fail
mov sync_count_num,R6
ajmp buf_exch_end

comm_sync_half:
mov B,R6
mov A,R7
cpl A
cjne A,B,data_check_fail
mov sync_half_num,R6
ajmp buf_exch_end

comm_sync_mult:
mov B,R6
mov A,R7
cpl A
cjne A,B,data_check_fail
mov sync_start_mult,R6
ajmp buf_exch_end

data_check_fail:
ajmp comm_error

