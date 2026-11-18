# Аудит повторного обучения и фильтра корпуса

Основной отчёт с **улучшениями и регрессиями**:
[`docs/VISION_IMPROVEMENT.md`](../../../../docs/VISION_IMPROVEMENT.md).

- `final.csv`, `final-args.yaml`, `final-training.json` — фактические 10 эпох,
  best=2, исходные метаданные обучения без runtime-оценки test.
- `dataset.json`, `review.json`, `dataset-recipe.py.txt` — снимок набора и
  точного рецепта при обучении. Сверяется SHA рецепта из снимка. Это
  агентская переработка TRAIN-псевдометок, не независимая ground truth.
- `rebuild-verification.json` — актуальный builder (ещё сбрасывает cache)
  воспроизводит тот же fingerprint всех обучающих пикселей/меток/split.
- `interrupted-*` — CSV/аргументы двух прерванных ранних экспериментов,
  причины в `interrupted-experiments.json`; веса не опубликованы и не
  использованы для старта финального обучения.
- `val-{baseline,improved}-{legacy,body,verified}.json` — все предсказания
  канонической val при одинаковых .25/512/preferRed=False, без ансамбля.
- `validation-selection.json` — исходный выбор по канонической val до
  обнаружения регрессии на полном val-ролике обуви.
- `profile-before-body-guard.json`, `test-improved-legacy.json`,
  `test-comparison-legacy.json` — сохранённый первый профиль и его test.
- `validation-selection-body-guard.json` — окончательный **val-only** выбор:
  прежняя цель macro-F1/цвета, false-green=0 на val, и не больше кадров с
  ложным светофором на отдельном 98-кадровом val-негативе, чем у baseline.
  Лучшие цвета не означают лучшие негативы: исходные веса+body дали 0
  ложных кадров обуви, новые+body — 11. Компромисс описан в отчёте.
- `validation-negative-guard-traces.json` — полный val-негатив для фильтров.
- `test-baseline-legacy.json` — точное воспроизведение сохранённых baseline
  TP/FP/FN и цветов. Оригинальный baseline JSON не перезаписан.
- `test-comparison.json` — итоговый профиль с body, зафиксированный до
  **повторной** оценки test. Этот test уже известен: не новый blind holdout.
  Никакого подбора по test после этого результата не выполнялось.
- `integration-smoke.json` — реальные веса через VisionService, backend
  `yolo+body`, совпадение с сохранённым предсказанием, без камер и привода.
- `checks.json`, `full-tests.txt` — последние проверки; общий pytest имеет
  известный сбой Kinect и наблюдавшийся перемежающийся сбой callback updater,
  изолированный повтор updater прошёл. Эти файлы проекта не изменялись.
- `*.py.txt` — исходники выполненных вспомогательных batch-прогонов для
  аудита, не самостоятельные команды из этой папки. Основные CLI находятся
  в `scripts/` и приведены в отчёте.

Три зелёных test-объекта, псевдометки и один коррелированный негативный
ролик не доказывают безопасность. Рабочая модель, моторы и E-STOP не менялись.
