Skrzyżowania

<tak, jak ja to widzę>

Skrzyżowanie samo w sobie jest serwerem, który publikuje Topiki, jak i do nich się subskrybuje.

Mapa powinna wysłać do mnie rozkaz pod tytułem "add_crossing" w parametrze którego jest zawarty obiekt typu "cross_msg". W ten sposób powiadamia mnie o tym, iż stworzyła w swoim świecie kolejne skrzyżowanie i przekazuje najważniejsze dla mnie informacje. Strukturę "cross_msg" deklaruje @Mateusz.

Najważniejszym zadaniem serwera skrzyżowań jest utworzenie Topików nazwanych "crossing_n", gdzie 'n' to numer ID danego skrzyżowania. Do skrzyżowań tych subskrybują się samochody, gdy chcą przez nie przejechać. Na tych kanałach serwer skrzyżowań wysyła odpowiednie odpowiedzi.

I tak, jeśli chodzi o komunikację : samochód <=> skrzyżowanie mamy do dyspozycji dwie klasy komunikatów:

	cross_request - Jest to zapytanie/informacja wysyłane do skrzyżowania. Znaczenie pól:
		carId - ID samochodu, który nadaje ten komunikat do skrzyżowania
		requiredDirection - kierunek, w jakim chce pojechać auto. numeracja kierunków : N(0), W(1), S(2), E(3), ALBO "-1" gdy chce się spytać skrzyżowania o możliwe skręty.
		isAfterCrossing - ma znaczenie tylko w przypadku, gdy auto przejechało już skrzyżowanie. Informuje w ten sposób skrzyżowanie, iż zwolniło się miejsce.


	cross_response - Jest to odpowiedź skrzyżowania na request samochodu. Znaczenie pól:
		availableDirections[] - tablica możliwych skrętów (zgodnie z numeracją kierunków świata kilka linijek wyżej). 
		previousAutoID - ID auta, które jechało też w tym kierunku. Pomoże to samochodowi ocenić, czy czasem na niego nie wjedzie.
		nextCrossID - informacja dla samochodu, jeśli już pokonał skrzyżowanie, jaki jest ID skrzyżowania, do którego następnie dojedzie.
		distanceToNextCross - informacja dla auta, jeśli już pokonał skrzyżowanie, jaki dystans dzieli go do następnego skrzyżowania.


	<JAK TO MA DZIAŁAĆ?>

1) Auto wysyła cross_request z polem 'required_direction' ustawionym na (-1)
	1.b) Dopóki skrzyżowanie nie odeśle odpowiedzi, auto staje.
2) Skrzyżowanie odsyła cross_response z wypełnionymi polami 'availableDirections'
3) Auto podejmuje decyzję o tym, gdzie chce pojechać
4) Auto wysyła ponownie cross_request z ustawionym kierunkiem, gdzie chce jechać.
	4.b) Dopóki skrzyżowanie nie odeśle odpowiedzi, auto staje
5) Skrzyżowanie zaklepuje miejsce dla tego auta na skrzyżowaniu w ramach tego skrętu. Jednocześnie odsyła cross_response z wypełnionymi polami "nextCrossID", "distanceToNextCross" oraz "previousID"
6) Skręt na danym skrzyżowaniu jest tak długo zajęty, dopóki auto nie poinformuje o jego pokonaniu. Czyni to poprzez ponowne wysłanie "cross_request" z polem "isAfterCrossing" ustawionym na TRUE


