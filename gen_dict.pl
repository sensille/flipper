#!/usr/bin/perl -w

use strict;
use Compress::Zlib;

open(FH, "<flipper.dict") or die;
my @st = stat(FH) or die;
my $sz = $st[7];
my $dict;
if (read(FH, $dict, $sz) != $sz) {
	die "failed to read input";
}
close FH;

my $z = Compress::Zlib::compress($dict, 9);
my $out = "";

for my $i (0..length($z)-1) {
	$out .= sprintf "%02x", ord(substr($z, $i, 1));
	if (($i % 16) == 15) {
		$out .= "\n";
	} elsif ($i != length($z) - 1) {
		$out .= " ";
	} else {
		$out .= "\n";
	}
}
open(FH, ">identify.mem") or die;
print FH $out;
close FH;
print "dict length is ".length($z)."\n";
