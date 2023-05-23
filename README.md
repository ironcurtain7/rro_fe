Команда Роботикс (Robotx). Описание робота для соревновний FutureEngineers 2023.

Робот был сделан на основе самостоятельно разработанного и напечатанного на 3D-принтере шасси. 3D модели для печати находятся в папке models. 
Вариант самостоятельной разработки шасси был выбран по двум причинам: 
- возможность сделать шасси нужной конфигурации для оптимального расположения электронных и механических компонентов робота 
- уменьшения массы робота и улучшенния маневренности по сравнению с готовыми радиоуправляемыми шасси

Система перемещения робота:

Шасси робота состоит из следующих частей: 
непосредственно рамы шасси (car_frame.stl), 
креплений мотора, рулевого сервопривода и камеры (motor_mount.stl, servo_mount.stl,camera_mount.stl),
дисков колес и вылитых из силикона шин (rim.stl, wheels1.stl, wheels2.stl)
Шины решили делать из силикона, т.к. можно сделать любую необходимую форму шины и выбрать мягкость шины для максимального сцепления с поверхностью.
А так же используется передний мост от радиоуправляемой модели автомобиля размера 1/24 для того, чтобы сделать робота переднеприводным для улучшения маневренности. В мосту стоит 
червячный редуктор для уменьшения передаваемых на колеса оборотов от двигателя.

Основной двигатель: Readytosky rs2205, бесколлекторный, размера 2205, 2200 оборотов. Управляется через регулятор двигателя (модель LittleBee 30A-S) ШИМ-сигналом с ESP32. Возможен передний и задний ход.
Сервопривод поворота колес: Tower Pro MG90S, 9 грамм

Так как шасси было специально разработано для вышеуказанных компонентов, то все части шасси собираются вместе с помощью винтов и гаек M3/M4 без какой-либо дополнительной подготовки. 

Система питания:
В роботе используется одна батарея LiPo 2s (7.4V) 1100mAh. Подключена через выключатель. От нее через регулятор двигателя питается собственно двигатель. А так же через DC-DC 
преобразователь на 5В питается вся остальная электроника. Для сглаживания падения напряжения от работы двигателя в схеме предусмотрен конденсатор 16v, 2200мкФ, подключенный 
параллельно батарее.

Контроллеры и сенсоры:
В роботе используется два микроконтроллера: 
ESP32 - основной контроллер. Используется для выполнения всей логики программы, управления основным и рулевым приводом, обработка данных с датчиков.
OpenMV H7 - контроллер анализа изображения с камеры. Анализирует изображение, находит необходимые объекты и передает данные по найденным объектам в основной контроллер.
Передача происходит по Serial порту. Данные контроллеры были выбраны по причине малых размеров, низкого энергопотребления, большого количества портов для сенсоров
 и достаточной для данной задачи производительности.

Используемые сенсоры:
- лазерный дальномер VL53L0X - используется для определения расстояния до стен полигона. Установлены по одному справа и слева в передней части робота.
- фототранзистор со светодиодной подствекой - используется для определения цвета линий на поверхности полигона путем анализа яркости отражаемого света. Т.е. яркость отраженного света
от линий белого, синего и оранжевого цвета различается и на этой основе определяется цвет линий.
- кнопка старта робота

Схема подключения системы питания к контроллерам, двигателям, а так же сенсоров к контроллеру находится в папке schemes (scheme.jpg).
Для уменьшения количества проводов между контроллерами и датчиками была спроектирована в программе SprintLayot и вытравлена на текстолите плата, на которую уже паялись разъемы для контроллеров и подключались датчики.
Ее схема так же находится в папке schemes (Плата FE.lay6).
 

Робот включается с помощью выключателя на кабеле к батарее. После этого он инициализирует регулятор/основной двигатель (издается звуковой сигнал) и ждет нажатия на кнопку старта.


Логика работы программы:

Квалификационный заезд (робот устанавливается на поле строго вдоль бортов, основная программа начинает работать после нажатия кнопки старта):

1. Робот определяет свою позицию относительно стенок полигона. Т.е. определяет расстояние до стены справа и слева с помощью дальномеров.
2. Двигается вперед строго прямо до нахождения синей/оранжевой полосы с помощью датчика яркости света
3. Если найдена синяя/оранжевая полоса, то в зависимости от ранее определенной позиции, робот едет с поворотом колес налево/направо на нужный угол необходимое количество 
времени таким образом, чтобы подъехать к перпендикулярной стене на расстояние примерно 20см и встать вдоль нее. Угол и время проезда подбирается опытным путем.
4. Затем запускается блок программы езды вдоль внешней стены полигона с помощью дальномеров и PID-регулятора. Робот едет на расстоянии 20см от внешней стены. Т.е. если первый поворот был налево 
и внешняя стенка справа, то едем по правому дальномеру. И наоборот.
5. Во время езды вдоль стены анализируем показания датчика яркости и определяем наезд на синюю/оранжевую линию.
6. Если наехали на синюю/оранжевую линию, то робот поворачивает колеса на нужный угол налево/направо и проезжает неободимое количество времени для того, чтобы подъехать к 
перпендикулярной стене на расстояние 20см и встать вдоль нее. Угол и время проезда подбирается опытным путем.
7. Затем алгоритм повторяется с п.4 до тех пор пока количество поворотов не достигнет 12.
8. После проезда двенадцатого поворота проезжаем заранее подобранное время с заранее подобранным углом поворота колес и останавливаемся.

Финальный заезд:



Программа для ESP32 написана на языке C++, на OpenMV - на языке python. Исходный код находится в папке src.
qual.cpp - программа для квалификационного заезда на ESP32
final.cpp - программа для финального заезда на ESP32
main.py - программа для контроллера OpenMV H7

Программы для ESP32 используют стандартные библиотеки из фреймворка PlatformIO, компилируется и загружаются в контроллер в среде Microsoft Visual Studio Code.
Программа для OpenMV H7 загружается в контроллер с помощью стандартной среды разработки от контроллеров OpenMV. (Скачать можно по ссылке https://openmv.io/pages/download)



