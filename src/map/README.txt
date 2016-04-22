Mapa

Implementowany serwis pozwala pobrać aktualną konfigurację mapy.
Mapa tworzy wszystkie skrzyżowania zdefiniowane w pliku konfiguracyjnym.
Przed konfiguracją z pliku mapa nie udostępnia serwisu.



Plik konfiguracyjny

Należy podawać pełną ścieżkę.


format pliku konfiguracyjnego:

(id skrz)
	(id N) (id E) (id S) (id W)
	(le N) (le E) (le S) (le W)
.
.
.
0 (koniec pliku)

zerowe id skrz. oznacza nieistniejące połączenie.

