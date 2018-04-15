# mvTracker #

## Список изменений ##

### 01.04.2018 ###
* Пост-обработка поля векторов теперь делается с задержкой, чтобы использовать последующий(ие?) кадры
* Перед морфологической обработкой выполняется обработка во временной области (_2009 ESTIMATING MOTION RELIABILITY TO IMPROVE MOVING OBJECT DETECTION IN THE H.264/AVC DOMAIN_). Параметры пока постоянные в **MoveDetector::DetectForeground()**
* Вывод в текст - результаты временной обработки (разные символы для решений на разных этапах), в файл - найденные маски после временной обработки + морф.
* Оптимизаций не завезли (опять), но производительность просела минимально

### 08.04.2018 ###

* Получение и обработка векторного поля в разрешении 4х4 (максимально возможном). 
  - Интерполяция для интра-блоков не делалась, ибо их на тестовых последовательностях сравнительно немного
  - Об оптимизации тоже никто долго не думал
  - Выигрыш в разделении областей есть, но в них появилось больше дыр и больше мелких ложных срабатываний
  - Флаг **-g 0** включает разрешение 16х16, **-g -1** включает 4х4. Остальные значения пока работают криво
* Для увеличившихся массивов размер стека увеличен до 32 МБ. В дальнейшем стек нужно будет освободить от всего большого

### 15.05.2018 ###

* Зайчатки трекера блобов
  - Шаг 1 - установить наличие/отстуствие блоба в следующем кадре. Работает пока плохо, нужно настраивать дальше
  - Шаг 2 - пройти по соседним кадрам и восстановить пропуски и удалить быстро исчезающие области
  - ???
* Вывод в цветной *.y4m