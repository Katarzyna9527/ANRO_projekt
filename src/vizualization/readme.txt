wcze�niej musi by� odpalona mapa, je�li nie to co� si� popsuje

odpala si� vizualization_node

odpala si� rviz, zmienia pole Fixed Frame na "/my_frame"
dodaje si� (Add) MarkerArray, topic: visualization_marker_array

do przetestowania suchego mo�na u�y� polecenia 
rostopic pub /viz_auto vizualization/auto_viz autoID startCrossID endCrossID distance

powinno by� przynajmniej prawid�owe z map�, takie jest w sumie za�o�enie,
gdzie� w dokumentacji jest wi�cej napisane o komunikacji