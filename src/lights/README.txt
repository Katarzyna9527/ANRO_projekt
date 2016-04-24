ŒWIAT£A - o co kaman:

DirectionAllower - klasa bêd¹ca sygnalizacj¹ œwietln¹, zmienia swój stan co 5 sekund, poprzez przesuwanie wierszy macierzy w lewo b¹dŸ w prawo
w zale¿noœci od tego, jaki to jest wiersz. Oto jak siê to powinno prezentowaæ:
     N               Je¿eli mamy sytuacjê, w której samochody z kierunków N i S maj¹ pozwolenie na skrêt w lewo (tak¿e nawrót), to wówczas w macierzy w klasie DA
    | |              bêdzie to reprezentowane tak:            SK¥D
 ___| |____                                              D   N E S W
W___   ____E                                             O N 1 0 0 0
    | |                                                  K E 1 0 0 0
    | |           					 ¥ S 0 0 1 0
     S							 D W 0 0 1 0

Nastêpny stan otrzymujemy po przesuniêciu wierszy 0 i 2 w lewo (z przeniesieniem elementu z pocz¹tku na koniec), natomiast 1 i 3 w prawo (równie¿ z przeniesieniem)
Wobec tego nastêpny stan to:
     N               
    | |                         			      SK¥D
 ___| |____                                              D   N E S W
W___   ____E                                             O N 0 0 0 1
    | |                                                  K E 0 1 0 0
    | |           					 ¥ S 0 1 0 0
     S							 D W 0 0 0 1

Odpowiada to sytuacji, w której samochody z kierunków E i W mog¹ skrêciæ w lewo, albo nawróciæ. Po kolejnej iteracji:
     N               
    | |                         			      SK¥D
 ___| |____                                              D   N E S W
W___   ____E                                             O N 0 0 1 0
    | |                                                  K E 0 0 1 0
    | |           					 ¥ S 1 0 0 0
     S							 D W 1 0 0 0

Tym razem, mo¿emy z kierunków N i S, jechaæ prosto, albo skrêciæ w prawo. Kolejna iteracja, da nam mo¿liwoœæ przejazdu z W i E na wprost b¹dŸ w prawo
