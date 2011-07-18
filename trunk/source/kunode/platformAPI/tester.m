g++ -c Platform.cpp -I ./ -o Platform.o
g++ Tester.cpp Platform.o rlserial.o -I ./ -o Tester

