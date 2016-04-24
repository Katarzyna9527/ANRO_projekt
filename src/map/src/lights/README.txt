�WIAT�A - o co kaman:

DirectionAllower - klasa b�d�ca sygnalizacj� �wietln�, zmienia sw�j stan co 5 sekund, poprzez przesuwanie wierszy macierzy w lewo b�d� w prawo
w zale�no�ci od tego, jaki to jest wiersz. Oto jak si� to powinno prezentowa�:
     N               Je�eli mamy sytuacj�, w kt�rej samochody z kierunk�w N i S maj� pozwolenie na skr�t w lewo (tak�e nawr�t), to w�wczas w macierzy w klasie DA
    | |              b�dzie to reprezentowane tak:            SK�D
 ___| |____                                              D   N E S W
W___   ____E                                             O N 1 0 0 0
    | |                                                  K E 1 0 0 0
    | |           					 � S 0 0 1 0
     S							 D W 0 0 1 0

Nast�pny stan otrzymujemy po przesuni�ciu wierszy 0 i 2 w lewo (z przeniesieniem elementu z pocz�tku na koniec), natomiast 1 i 3 w prawo (r�wnie� z przeniesieniem)
Wobec tego nast�pny stan to:
     N               
    | |                         			      SK�D
 ___| |____                                              D   N E S W
W___   ____E                                             O N 0 0 0 1
    | |                                                  K E 0 1 0 0
    | |           					 � S 0 1 0 0
     S							 D W 0 0 0 1

Odpowiada to sytuacji, w kt�rej samochody z kierunk�w E i W mog� skr�ci� w lewo, albo nawr�ci�. Po kolejnej iteracji:
     N               
    | |                         			      SK�D
 ___| |____                                              D   N E S W
W___   ____E                                             O N 0 0 1 0
    | |                                                  K E 0 0 1 0
    | |           					 � S 1 0 0 0
     S							 D W 1 0 0 0

Tym razem, mo�emy z kierunk�w N i S, jecha� prosto, albo skr�ci� w prawo. Kolejna iteracja, da nam mo�liwo�� przejazdu z W i E na wprost b�d� w prawo
