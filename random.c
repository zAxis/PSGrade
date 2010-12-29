char random_byte;

char rand(void)   {
   char sum;
   
   sum = 0;
   
   // This calculates parity on the selected bits (mask = 0xb4).
   if(random_byte & 0x80)
      sum = 1;
   
   if(random_byte & 0x20)
      sum ^= 1;
   
   if(random_byte & 0x10)
      sum ^= 1;
   
   if(random_byte & 0x04)
      sum ^= 1;
   
   random_byte <<= 1;
   
   random_byte |= sum;
   
   return(random_byte);
}

void srand(char seed) {
   random_byte = seed;
}
