

def get_map_url(polyline, coords):
    return ('https://maps.googleapis.com/maps/api/staticmap?size=400x400&path=weight:3%7Ccolor:blue%7Cenc:' +
            polyline +
            '&markers=icon:https://i.imgur.com/DD11yLv.png|color:blue%7Clabel:B%7C'+
            '{0},%20{1}'.format(*coords))


def get_map(polyline, coords):
    pass


def main():
    polyline = ('ezynEhmupUsKbGwJ_JyGkHrAbCk|@~iAm~@lhDka@``Bwe@|r@{l@tn@sb@|V}M`[kJdg@el@bs@k}@hc@o}@zaA{a@hiA{m@b_@' +
        'wo@xo@}oAhf@ut@tz@kh@pZgY|Uim@fF{lA|TsnBt]ep@kHi~CfeDyhBpzA_{@`q@ilA~xAwpBbeBkn@fx@}b@tf@ks@`k@s_@ny' +
        'Aq`@bo@_ZhcAei@~r@qc@hMchBvcA}jD`{@s~Bz|@k}@bcAkt@xxAc^vUoa@fCodAxC}eAkL}j@nQwiApt@ew@hl@c~@tu@_y@dj' +
        '@{aAlm@kw@r_Aox@xg@if@p|@ay@ln@mYvx@kv@||@{x@vIw|ArAmyAjk@{b@~i@k^tl@uPpbAeWla@}[fMakAxRag@bg@cf@d\cTrq@gj@ff@e|A`' +
        'bAkpAba@sbBrNexBaRo_BT}qAly@qrAzdCyc@zs@iLbkAiRxjCsl@by@s`@j^uU^ecA{}@wqAwfAezAb{@otAjdA_dAbiAqe@n]k' +
        'hA|tB_UxQik@~GueA`B{t@iJ{mAxi@_cBpd@wpEdaAgpDrn@}nDn}Di}Wx`ZmlTf_Vw|B`kC_wCtlEywEfqHohXn{a@_iFteGwhI' +
        '|}IqtEv_EqhAru@}nFjqDg_JfeG}_TjuNswE~wDiqF`kFutQr|QufAhsA}h@d`Amw@lvAkpB`hEyhEdqIwlB`oBetGh|FupGrtFs' +
        'mItqE_`Jn}EkzAjaAcpCjbCmoHvnG}kStmQmnBtfB{tCd`Du~LloQs`D~tEyqB~{Ak|HjrFwkKb}GkgYraQaeChsB}cAfjAcxEd}' +
        'F_vCz`DijB||AywE`nEg~EbwE_sAlpAkdEljDyaI`rEspEveCe_Fry@}mFd|@w|HdtAkpG`xAwtBlh@w_B`bAy|B~kAasDhhBgeB' +
        'xpAk|A||A{sBpaCmjBfbCmmCbkDquLnxP{dCddE_cE~aIuvHzaOaeB`gCseAlqA{Kz]fEj{Adn@f_Ije@|h@tJjn@vp@blCsSdt@' +
        'vCvqAfw@bwD~w@t{CjI`eEGbyBkDxhQs@bwDdKx~@vHxt@zDprDeQ~tCoShpAhI`lBs@xoAbHzg@ri@h{@vNfuBdC`cB^rnAgLn`' +
        '@ei@|d@af@z|@{p@nnA_{@vd@icArRigBhjAie@gU_TNmw@xd@_cAjoB_Vnu@gL|sAep@nbAkk@vrC}^fp@}WpdAmw@de@ce@dsB' +
        'mLt`AnCjj@mF~YtQt|Adj@`tGz_A|uAdsBx{BztA`rBz`@c@dCbZeJxmAeWuS&key=AIzaSyDF-CtL_mHkQg2OKRh6aS5tYELp1B1zWIw')
    coords = (35.996230, -118.996203)
    print(get_map_url(polyline, coords))

if __name__ == '__main__': main()
