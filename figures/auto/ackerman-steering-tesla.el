(TeX-add-style-hook
 "ackerman-steering-tesla"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-class-options
                     '(("standalone" "margin=10pt")))
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("fontenc" "T1") ("inputenc" "utf8")))
   (TeX-run-style-hooks
    "latex2e"
    "standalone"
    "standalone10"
    "fontenc"
    "inputenc"
    "pgf"
    "tikz")
   (TeX-add-symbols
    '("wheel" 2)
    "largejoint"
    "joint"
    "track"
    "wbase"
    "wwidth"
    "wrad"
    "linkl"
    "trail"))
 :latex)

