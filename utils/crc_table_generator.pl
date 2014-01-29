#!/usr/bin/perl

use strict;
use warnings;

# Width of the CRC polynomial 
my $width    = 16;
# Top bit index of a $width bit wide variable
my $top_bit  = (1 << ($width - 1));
# 16-bit binary mask 
my $bit_mask = 0xFFFF;  

my ($polynomial, $file) = @ARGV;
if($polynomial =~ m/0x/) {
    $polynomial =~ s/0x(.*)/$1/;
    $polynomial = hex($polynomial);
}

my @crc_table = (); 

for(my $dividend = 0; $dividend < 256; $dividend++) {
    
    my $remainder = $dividend << ($width - 8);
    
    for(my $bit = 8; $bit > 0; $bit--) {
        
        if(($remainder & $top_bit) != 0) {
            $remainder = (($remainder << 1) & $bit_mask) ^ $polynomial;
        }
        
        else {
            $remainder = (($remainder << 1) & $bit_mask);
        }
    }
    
    $crc_table[$dividend] = $remainder; 
}

# Write data to the output file

# Create the output file
open(OUTFD, '>>', $file)
    or die("Couldn't create file $file: $!");      # Create the output file
open(OUTFD, '>', $file);

my $header = "/*\n * Automatically generated file: do not modify.\n * CRC polynomial: $ARGV[0].\n */";

# Print comment header.
print OUTFD $header, "\n\n"; 

# Print CRC table as a C array.
my $line_counter = 8;

print OUTFD 'const uint8_t crc_table[256] = {', "\n\t"; 
for(my $index = 0; $index < 256; $index++) {
    printf OUTFD "0x%04xu, ", $crc_table[$index];
    $line_counter--;

    if(($line_counter == 0) && ($index < 256)) {
        print OUTFD "\n\t"; 
        $line_counter = 8; 
    }
}
print OUTFD "\n};";

close(OUTFD);

