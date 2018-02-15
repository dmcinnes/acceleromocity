
void readAccelData(int *destination);
void updateAccelData();
void interruptHandler();
void goToSleep();
float accelerationDotProduct(float x, float y);

/* Routines */
void followSide();
void followSingle();
void followHorizon();
void followMarquee();
void chase();
void doubleChase();
void twinkle();
void fade();
void fadeCycle();
void twinkleFade();
