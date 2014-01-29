#!/usr/bin/perl

use strict;
use warnings;

my ($polynomial) = @ARGV;
if($polynomial =~ m/0x/) {
    $polynomial =~ s/0x(.*)/$1/;
    $polynomial = hex($polynomial);
}

$polynomial += 2; 
print "Polynomial: $polynomial\n"; 

