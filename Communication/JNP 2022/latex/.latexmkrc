$latex = 'latex  %O  --shell-escape %S';
$pdflatex = 'lualatex  %O  --shell-escape %S';

$pdf_mode = 1;

add_cus_dep( 'abo', 'abi', 0, 'makeglossaries' );
sub makeglossaries {
   my ($base_name, $path) = fileparse( $_[0] );
   pushd $path;
   my $return = system "makeglossaries", $base_name;
   popd;
   return $return;
}

add_cus_dep("nlo", "nls", 0, "nlo2nls");
sub nlo2nls {
    system("makeindex $_[0].nlo -s nomencl.ist -o $_[0].nls -t $_[0].nlg");
}

$clean_ext .= " abr abi abo nlo nls nlg ist ptc auxlock ";