\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{tcthesis}[15/04/2020 Written by T. Chevet -- Ph.D. Thesis class]

\newif\if@frame \@framefalse
\newif\if@extfig \@extfigfalse
\newif\if@fr \@frfalse
\newif\if@print \@printfalse
\newif\if@times \@timesfalse
\newif\if@list \@listtrue

% This option is used to display the margins
\DeclareOption{dispframe}{\@frametrue}
% This option is used to externalize the compilation of the figures
\DeclareOption{extfigures}{\@extfigtrue}
% This option is used to signify that main language is French
\DeclareOption{fr}{\@frtrue}
% This option is used to signify that two languages are used
\DeclareOption{printable}{\@printtrue}
% This option is used to use Times New Roman and mtpro2
\DeclareOption{times}{\@timestrue}
% This option is used to remove Chapter from additional lists
\DeclareOption{nochapterinlist}{\@listfalse}

% All locally unknown options are passed to memoir
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{memoir}}

\ProcessOptions\relax

\LoadClass{memoir}

\RequirePackage{luacode}

\RequirePackage[no-math]{fontspec}

\if@times
	\setmainfont{Times New Roman}
\else
\fi

\RequirePackage{polyglossia}

\if@fr
	% \RequirePackage[main = french, english]{babel}
	\setdefaultlanguage{french}
	\setotherlanguages{english, spanish}
\else
	% \RequirePackage[main = english, french]{babel}
	\setdefaultlanguage{english}
	\setotherlanguages{french, spanish}
\fi

\RequirePackage{etoolbox}
\RequirePackage{xpatch}
\RequirePackage{xparse}
\RequirePackage[%
	backend      = biber,%			 % Biber chosen as bibliography engine
	maxbibnames  = 99,%              % Display all authors of entry in bibliography
	maxcitenames = 2,%               % Display ``et al.'' when a cited entry is authored by more than two people
	mincitenames = 1,%               % Display ``et al.'' when a cited entry is authored by more than two people
	uniquelist   = false,%
	uniquename   = false,%
	nohashothers = true,%
	sortcites    = true,%
	sorting      = ynt,%
	bibstyle     = authoryear,%      % Default authoryear biblatex style for bibliography
	citestyle    = authoryear-comp,% 
	backref      = true,%            % List all pages in which bibliography entry is cited
	dashed       = false,%
]{biblatex}

% The amsmath package is a LaTeX package that provides miscellaneous enhancements for improving the information structure and printed output of documents that contain mathematical formulas
\RequirePackage{amsmath}
% Mathtools provides a series of packages designed to enhance the appearance of documents containing a lot of mathematics
\RequirePackage{mathtools}
% The package facilitates the kind of theorem setup typically needed in American Mathematical Society publications
\RequirePackage{amsthm}
% The bundle provides several packages for commonly-needed support for typesetting theorems
\RequirePackage{thmtools}

\if@times
	\let\heavymath\relax
	\RequirePackage[mtphrb,mtpscr,subscriptcorrection]{mtpro2}

	\patchcmd{\PEX@}{\dp\Pbox@>\dp\z@}{\ht\Pbox@>\dp\z@}{}{}
	\patchcmd{\SQEX@}{\dp\Sbox@>\dp0}{\ht\Sbox@>\dp0}{}{}

	\DeclareRobustCommand\{{\ifmmode\lbrace\else\textbraceleft\fi}
	\DeclareRobustCommand\}{\ifmmode\rbrace\else\textbraceright\fi}

	\begin{luacode}
		function nabla_f ( s )
			return ( s:gsub ( "\\nabla%s*f" , "\\nabla\\mkern-4mu f" ) )
		end
    \end{luacode}
	\directlua{luatexbase.add_to_callback("process_input_buffer", nabla_f , "nablaf" )}
\else
	% amssymb provides an extended symbol collection
	\RequirePackage{amssymb}
	% Support use of the Raph Smith’s Formal Script font in mathematics. Provides a \mathscr command, rather than overwriting the standard \mathcal command, as in calrsfs
	\RequirePackage{mathrsfs}

	% Import widecheck from mathabx package without wasting the other math fonts
	\DeclareFontFamily{U}{mathx}{\hyphenchar\font45}
	\DeclareFontShape{U}{mathx}{m}{n}{
	      <5> <6> <7> <8> <9> <10>
	      <10.95> <12> <14.4> <17.28> <20.74> <24.88>
	      mathx10
	      }{}
	\DeclareSymbolFont{mathx}{U}{mathx}{m}{n}
	\DeclareFontSubstitution{U}{mathx}{m}{n}
	\DeclareMathAccent{\widecheck}{0}{mathx}{"71}

	% The bm package defines a command \bm which makes its argument bold
	\RequirePackage{bm}
\fi

\RequirePackage[most]{tcolorbox}
\if@extfig
	\tcbsetforeverylayer{shield externalize}
\fi
\RequirePackage{varwidth}

\RequirePackage{pdfpages}

\RequirePackage{hyphenat}

% A companion for titlesec handling toc/lof/lot entries
\RequirePackage{titletoc}

% The package builds upon the graphics package, providing a key-value interface for optional arguments to the \includegraphics command
\RequirePackage{graphicx}

% Algorithm2e is an environment for writing algorithms
\RequirePackage[ruled, linesnumbered, algochapter]{algorithm2e}

% This small library provides a standard set of environments for writing optimization problems
\RequirePackage[c1]{optidef}

% This package can disable all hyphenation or enable hyphenation of non-alphabetics or monospaced fonts
\RequirePackage{hyphenat}
% The xspace package provides a single command that looks at what comes after it in the command stream, and decides whether to insert a space to replace one "eaten" by the TeX command decoder
\RequirePackage{xspace}

\RequirePackage{bold-extra}

% The glossaries package supports acronyms and multiple glossaries
\RequirePackage[nomain, toc, nopostdot]{glossaries}
% Produces lists of symbols using the capabilities of the MakeIndex program
\RequirePackage[intoc]{nomencl}

% This package extends the ifthen package by implementing new commands to go within the first argument of \ifthenelse: to test whether a string is void or not, if a command is defined or equivalent to another
\RequirePackage{xifthen}

\RequirePackage{array}
\RequirePackage{multirow}
\RequirePackage{booktabs}

% The package provides a LaTeX interface to the micro-typographic extensions that were introduced by pdfTeX and have since also propagated to XeTeX and LuaTeX: most prominently, character protrusion and font expansion, furthermore the adjustment of interword spacing and additional kerning, as well as hyphenatable letterspacing (tracking) and the possibility to disable all or selected ligatures
\RequirePackage{microtype}

\RequirePackage{csquotes}

\RequirePackage{enumitem}

\RequirePackage{calc}
\RequirePackage{siunitx}

% PGF is a macro package for creating graphics
\RequirePackage{tikz}
	\usetikzlibrary{babel}
	\usetikzlibrary{calc}
	\usetikzlibrary{shapes.geometric}
	\usetikzlibrary{arrows.meta}
	\usetikzlibrary{positioning}
	\usetikzlibrary{patterns}
	\usetikzlibrary{matrix}
	\usetikzlibrary{3d}
	\usetikzlibrary{decorations.markings}
	\usetikzlibrary{fit}
	\usetikzlibrary{external}
	\if@extfig
		\newcommand{\figoutroot}{figures_out}
		\tikzexternalize[optimize command away=\includepdf]
	\fi
% PGFPlots draws high-quality function plots in normal or logarithmic scaling with a user-friendly interface directly in TeX
\RequirePackage{pgfplots}
	\usepgfplotslibrary{groupplots}
	\pgfplotsset{compat = newest}

\if@frame
	% The package provides an easy and flexible user interface to customize page layout, implementing auto-centering and auto-balancing mechanisms so that the users have only to give the least description for the page layout
	\RequirePackage[showframe]{geometry}
	\tikzifexternalizing{\setkeys{Gm}{showframe = false}}{}
	\RequirePackage{showlabels}
\fi

\if@print
	% The hyperref package is used to handle cross-referencing commands in LaTeX to produce hypertext links in the document
	\RequirePackage[breaklinks,hidelinks]{hyperref}
\else
	\RequirePackage[breaklinks]{hyperref}
		\hypersetup{
			colorlinks = true,
			citecolor  = blue,
			linkcolor  = blue,
			urlcolor   = blue
		}
\fi

\RequirePackage{cleveref}

\if@list
	\newcommand{\DeclareDividedList}[1]{%
		\newcounter{#1@chapter}%
		\counterwithin*{#1@chapter}{chapter}%
		\setcounter{#1@chapter}{0}%
	}

	\pretocmd{\addcontentsline}{%
		\ifltxcounter{#1@chapter}{%
			\ifnumgreater{1}{\value{#1@chapter}}{%
				\setcounter{#1@chapter}{1}%
				\addtocontents{#1}{\protect\contentsline{chapter}{\protect\chapternumberline{\thechapter}\f@rbdy}{}{chapter.\thechapter}}
			}{}%
		}{}%
	}{}{}

	\DeclareDividedList{lof}
\else
\fi

% Definition of commands to add partial table of contents for each chapters
% To use this, add \startcontents[chapters] and \Mprintcontents after the \chapter{} command and \stopcontents[chapters] at the end of the chapter
\newcommand\ToCrule{\noindent\rule[5pt]{\textwidth}{0.5pt}}
\newcommand\ToCtitle{{\large\bfseries\contentsname}\vskip2pt\ToCrule}
\newcommand\Mprintcontents{%
	\ToCtitle
	\vspace{-15pt}
	\printcontents[chapters]{}{1}{}\par\nobreak
	\ToCrule
}

\pretocmd{\chapter}{%
	\startcontents[chapters]
}{}{}

% This command is there to make the glossary links the same color as the surrounding text (better looking than having a fixed color in the middle of a title or a link)
\newcommand*{\glsplainhyperlink}[2]{%
	\colorlet{currenttext}{.}% store current text color
	\colorlet{currentlink}{\@linkcolor}% store current link color
	\hypersetup{linkcolor=currenttext}% set link color
	\hyperlink{#1}{#2}%
	\hypersetup{linkcolor=currentlink}% reset to default
}
\let\@glslink\glsplainhyperlink

% The following makes the header in normal font
\nouppercaseheads
\pagestyle{headings}
\makeheadrule{headings}{\textwidth}{0.5pt}

% The following modifies the madsen style of memoir class
\makechapterstyle{custommadsen}{% requires graphicx package
	\chapterstyle{default}
	\setlength{\beforechapskip}{0pt}
	\renewcommand*{\chapnamefont}{%
		\normalfont\LARGE\scshape\raggedleft%
	}
	\renewcommand*{\chaptitlefont}{%
		\normalfont\Huge\scshape\raggedleft%
	}
	\renewcommand*{\chapternamenum}{}
	\renewcommand*{\printchapternum}{%
		\makebox[0pt][l]{\hspace{0.4em}%
		\resizebox{!}{3ex}{%
			\chapnamefont\bfseries\thechapter}%
		}%
	}%
	\renewcommand*{\printchapternonum}{%
		\chapnamefont \phantom{\printchaptername \chapternamenum%
		\makebox[0pt][l]{
			\hspace{0.4em}%
			\resizebox{!}{4ex}{%
				\chapnamefont\bfseries 1}%
			}%
		}%
		\afterchapternum %
	}%
	\renewcommand*{\afterchapternum}{%
		\par\hspace{1.5cm}\hrule\vskip\midchapskip%
	}%
}
\chapterstyle{custommadsen}

% Add an abbreviation glossary
\newglossary[abr]{abbrv}{abi}{abo}{List of Acronyms}
\makeglossaries
\renewcommand{\glossarypreamble}{\glsfindwidesttoplevelname[\currentglossary]}

% Definition of the list of symbols with nomencl package
\newdimen\nomwidest
\renewcommand{\nomlabel}[1]{%
	\sbox\z@{#1\hspace{\labelsep}}%
	\ifdim\nomwidest<\wd\z@\global\nomwidest\wd\z@\fi
	#1\hfil\hspace{\labelsep}%
}
\renewcommand{\nompostamble}{%
	\protected@write\@auxout{}{\global\nomwidest=\the\nomwidest}%
}
\makenomenclature
\newcommand{\newprintnomenclature}{
	\renewcommand{\nomname}{List of Symbols}
	\markboth{\nomname}{\nomname}
	\printnomenclature[\nomwidest]
}

\addtocontents{lof}{\cftpagenumbersoff{chapter}}
\addtocontents{lasm}{\cftpagenumbersoff{chapter}}
\addtocontents{ldfn}{\cftpagenumbersoff{chapter}}

\if@list
	\DeclareDividedList{lasm}
	\DeclareDividedList{ldfn}
\else
\fi

\renewcommand*{\numberline}[1]{%
	\numberlinehook{#1}%
	\hb@xt@\@tempdima{\@cftn@me\@cftbsnum #1\@cftasnum\hfil}\@cftasnumb
}

\renewcommand*{\cftchaptername}{\chaptername~}
\renewcommand*{\cftfigurename}{\figurename~}
\renewcommand*{\cftfigureaftersnum}{:}
\setlength{\cftfigureindent}{\cftsectionindent}

\renewcommand{\insertchapterspace}{%
	\addtocontents{lof}{\protect\addvspace{0pt}}%
	\addtocontents{lot}{\protect\addvspace{0pt}}%
}

\setsecnumdepth{paragraph}
\settocdepth{subsubsection}

\setfloatadjustment{figure}{\centerfloat}
\captionstyle{\centering}
\changecaptionwidth
\captionwidth{0.9\textwidth}
\setfloatadjustment{table}{\centerfloat}

\newsubfloat{figure}

\setcounter{MaxMatrixCols}{100}

\sisetup{inter-unit-product = \ensuremath{{}\cdot{}}}

% Give more vertical spacing to a matrix
% Works by calling \begin{<type>matrix}[<value>] where <value> is an optional argument meant to modify the vertical spacing
\renewcommand*\env@matrix[1][\arraystretch]{%
	\edef\arraystretch{#1}%
	\hskip -\arraycolsep
	\let\@ifnextchar\new@ifnextchar
	\array{*\c@MaxMatrixCols c}
}

\strictpagecheck

% Command to avoid having numbers protruding in the right margin in tables of content
\setpnumwidth{2.2em}

%%%% Modifications for the bibliography style

% Command to print the final bibliography with entries sorted by first author name then year then title
\newcommand{\printgeneralbibliography}{%
	\begin{refcontext}[sorting = nyt]
		\printbibliography[
			notkeyword = {local}
		]
	\end{refcontext}
}

% Have a comma between the authors and the year with \parencite
\DeclareDelimFormat{nameyeardelim}{\addcomma\space}
% Sort with surname of first author and then with year
\DeclareSourcemap{
	\maps[%
		datatype = bibtex%
	]{
		\map[%
			overwrite = false%
		]{
			\step[%
				fieldsource = author,%
				match       = \regexp{(.+?)\s+and\s+(.+)},%
				final%
			]
			\step[%
				fieldset   = sortname,%
				fieldvalue = {$1$}%
			]
		}
	}
}
% Have all authors displayed as given-middle-family name in bibliography
\DeclareNameAlias{sortname}{given-family}
% Make surname of author small caps in the bibliography
\AtEveryBibitem{%
	\renewcommand*{\mkbibnamefamily}[1]{%
		\textsc{#1}%
	}%
}
\AtEveryCite{%
	\renewcommand*{\mkbibnamefamily}[1]{%
		\textnormal{#1}%
	}%
}
% Remove quotation mark around title in bibliography
\DeclareFieldFormat{title}{#1\isdot}
% Change backref text in bibliography
\DefineBibliographyStrings{english}{%
	backrefpage = {cited on p\adddot},%
	backrefpages = {cited on pp\adddot},%
}
% Put the issue number in parentheses and embolds it
\xpatchbibmacro{volume+number+eid}{%
	\setunit*{\adddot}%
}{%
}{}{}
\DeclareFieldFormat[article]{number}{\mkbibparens{\textbf{#1}}}










\providecommand{\prfname}{}
\providecommand{\Prfname}{}
\providecommand{\prfnames}{}
\providecommand{\Prfnames}{}

\providecommand{\dfnname}{}
\providecommand{\Dfnname}{}
\providecommand{\dfnnames}{}
\providecommand{\Dfnnames}{}

\providecommand{\thmname}{}
\providecommand{\Thmname}{}
\providecommand{\thmnames}{}
\providecommand{\Thmnames}{}

\providecommand{\lmmname}{}
\providecommand{\Lmmname}{}
\providecommand{\lmmnames}{}
\providecommand{\Lmmnames}{}

\providecommand{\crlname}{}
\providecommand{\Crlname}{}
\providecommand{\crlnames}{}
\providecommand{\Crlnames}{}

\providecommand{\prpname}{}
\providecommand{\Prpname}{}
\providecommand{\prpnames}{}
\providecommand{\Prpnames}{}

\providecommand{\rmkname}{}
\providecommand{\Rmkname}{}
\providecommand{\rmknames}{}
\providecommand{\Rmknames}{}

\providecommand{\asmname}{}
\providecommand{\Asmname}{}
\providecommand{\asmnames}{}
\providecommand{\Asmnames}{}

\providecommand{\exmpname}{}
\providecommand{\Exmpname}{}
\providecommand{\exmpnames}{}
\providecommand{\Exmpnames}{}

\providecommand{\lodname}{}

% This command is used to declare the name of a thmtools theorem environment
% It is used with \dtname{<command containing the environment name without the \>} 
\newcommand{\dtname}[1]{%
	\expandafter\noexpand
	\expandafter\noexpand
	\expandafter\noexpand
	\csname #1\endcsname
}