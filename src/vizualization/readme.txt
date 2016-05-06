wczeœniej musi byæ odpalona mapa, jeœli nie to coœ siê popsuje

odpala siê vizualization_node

odpala siê rviz, zmienia pole Fixed Frame na "/my_frame"
dodaje siê (Add) MarkerArray, topic: visualization_marker_array

do przetestowania suchego mo¿na u¿yæ polecenia 
rostopic pub /viz_auto vizualization/auto_viz autoID startCrossID endCrossID distance

powinno byæ przynajmniej prawid³owe z map¹, takie jest w sumie za³o¿enie,
gdzieœ w dokumentacji jest wiêcej napisane o komunikacji