param(
    [switch]$Silent
)

$ErrorActionPreference = "Stop"
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

$AppDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$EventsPath = Join-Path $AppDir "events.json"
$SourcesPath = Join-Path $AppDir "sources.json"
$SettingsPath = Join-Path $AppDir "settings.json"
$SeenPath = Join-Path $AppDir "seen.json"
$NotifiedPath = Join-Path $AppDir "notified.json"
$StatusPath = Join-Path $AppDir "status.json"
$LogPath = Join-Path $AppDir "monitor.log"
$CalendarPath = Join-Path $AppDir "RecruitCalendar.hta"

function Log([string]$Message) {
    $line = ("{0}  {1}" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss"), $Message)
    Add-Content -LiteralPath $LogPath -Value $line -Encoding UTF8
}

function Read-JsonFile([string]$Path, $DefaultValue) {
    try {
        if (Test-Path -LiteralPath $Path) {
            $raw = Get-Content -LiteralPath $Path -Raw -Encoding UTF8
            if ($raw.Trim().Length -gt 0) { return ($raw | ConvertFrom-Json) }
        }
    } catch { Log ("JSON read error: " + $Path + " / " + $_.Exception.Message) }
    return $DefaultValue
}

function Write-JsonFile([string]$Path, $Value) {
    $json = $Value | ConvertTo-Json -Depth 8
    [System.IO.File]::WriteAllText($Path, $json, (New-Object System.Text.UTF8Encoding($true)))
}

function Html-ToText([string]$Html) {
    if ([string]::IsNullOrWhiteSpace($Html)) { return "" }
    $t = [regex]::Replace($Html, "(?is)<script\b.*?</script>", " ")
    $t = [regex]::Replace($t, "(?is)<style\b.*?</style>", " ")
    $t = [regex]::Replace($t, "(?is)<br\s*/?>", "`n")
    $t = [regex]::Replace($t, "(?is)</(p|div|li|tr|h[1-6])>", "`n")
    $t = [regex]::Replace($t, "(?is)<[^>]+>", " ")
    $t = [System.Net.WebUtility]::HtmlDecode($t)
    $t = $t -replace "[\u00A0\t\r]+", " "
    $t = $t -replace " +", " "
    $t = $t -replace "(\n\s*){3,}", "`n`n"
    return $t.Trim()
}

function Get-Html([string]$Url) {
    $headers = @{
        "User-Agent" = "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 Chrome/124 Safari/537.36"
        "Accept-Language" = "ko-KR,ko;q=0.9,en-US;q=0.7,en;q=0.6"
    }
    $resp = Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec 25 -Headers $headers -MaximumRedirection 5
    return $resp.Content
}

function Repair-Mojibake([string]$Text) {
    if ([string]::IsNullOrEmpty($Text)) { return $Text }
    if ($Text -notmatch "[ÃÂêëìíîï]") { return $Text }
    try {
        return [Text.Encoding]::UTF8.GetString([Text.Encoding]::GetEncoding(28591).GetBytes($Text))
    } catch { return $Text }
}

function Invoke-KaiJobflexApi {
    $headers = @{
        "User-Agent" = "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 Chrome/124 Safari/537.36"
        "Accept-Language" = "ko-KR,ko;q=0.9,en-US;q=0.7,en;q=0.6"
        "prefix" = "koreaaero.recruiter.co.kr"
        "Origin" = "https://koreaaero.recruiter.co.kr"
        "Referer" = "https://koreaaero.recruiter.co.kr/career/job"
    }
    $payload = [ordered]@{
        pageableRq = [ordered]@{ page = 1; size = 50 }
        filter = [ordered]@{}
    }
    $body = $payload | ConvertTo-Json -Depth 6 -Compress
    return Invoke-RestMethod -Uri "https://api-recruiter.recruiter.co.kr/position/v1/jobflex" -Method POST -ContentType "application/json;charset=UTF-8" -Headers $headers -Body $body -TimeoutSec 25
}

function Resolve-Link([string]$BaseUrl, [string]$Href) {
    try {
        $Href = [System.Net.WebUtility]::HtmlDecode($Href).Trim()
        if ([string]::IsNullOrWhiteSpace($Href)) { return $null }
        if ($Href -match "^(javascript:|mailto:|tel:)") { return $null }
        if ($Href -match "^https?://") { return $Href }
        $baseUri = New-Object System.Uri($BaseUrl)
        $uri = New-Object System.Uri($baseUri, $Href)
        return $uri.AbsoluteUri
    } catch { return $null }
}

function Get-AttrValue([string]$Attrs, [string]$Name) {
    $m = [regex]::Match($Attrs, '(?is)\b' + [regex]::Escape($Name) + '\s*=\s*([''"])(?<v>.*?)\1')
    if ($m.Success) { return [System.Net.WebUtility]::HtmlDecode($m.Groups["v"].Value) }
    return ""
}

function Get-OnclickLink([string]$Onclick, [string]$BaseUrl) {
    if ([string]::IsNullOrWhiteSpace($Onclick)) { return "" }
    if ($Onclick -match "fn_search_detail\('(?<id>[^']+)'\)") {
        $bbs = "BBSMSTR_000000000793"
        if ($BaseUrl -match "/bbs/(?<bbs>BBSMSTR_[^/]+)/") { $bbs = $matches["bbs"] }
        return "/bbs/" + $bbs + "/view.do?nttId=" + $matches["id"]
    }
    if ($Onclick -match "_dku_bbs_web_BbsPortlet_viewMessage\((?<id>\d+)") {
        return "?p_p_id=dku_bbs_web_BbsPortlet&p_p_lifecycle=0&p_p_state=normal&p_p_mode=view&_dku_bbs_web_BbsPortlet_cur=1&_dku_bbs_web_BbsPortlet_action=view_message&_dku_bbs_web_BbsPortlet_orderBy=createDate&_dku_bbs_web_BbsPortlet_bbsMessageId=" + $matches["id"]
    }
    return ""
}

function Get-Anchors([string]$Html, [string]$BaseUrl) {
    $out = @()
    $matches = [regex]::Matches($Html, "(?is)<a\b(?<attrs>[^>]*)>(?<txt>.*?)</a>")
    foreach ($m in $matches) {
        $title = Html-ToText $m.Groups["txt"].Value
        if ([string]::IsNullOrWhiteSpace($title)) { continue }
        $attrs = $m.Groups["attrs"].Value
        $hrefRaw = Get-AttrValue $attrs "href"
        $onclickHref = Get-OnclickLink (Get-AttrValue $attrs "onclick") $BaseUrl
        if (-not [string]::IsNullOrWhiteSpace($onclickHref)) { $hrefRaw = $onclickHref }
        $href = Resolve-Link $BaseUrl $hrefRaw
        if ($null -eq $href) { continue }
        $out += [pscustomobject]@{ title = $title.Trim(); url = $href }
    }
    $buttons = [regex]::Matches($Html, "(?is)<button\b(?<attrs>[^>]*)>(?<txt>.*?)</button>")
    foreach ($m in $buttons) {
        $title = Html-ToText $m.Groups["txt"].Value
        if ([string]::IsNullOrWhiteSpace($title)) { continue }
        $attrs = $m.Groups["attrs"].Value
        $hrefRaw = Get-OnclickLink (Get-AttrValue $attrs "onclick") $BaseUrl
        $href = Resolve-Link $BaseUrl $hrefRaw
        if ($null -eq $href) { continue }
        $out += [pscustomobject]@{ title = $title.Trim(); url = $href }
    }
    return $out
}

function New-Id([string]$Value) {
    $bytes = [Text.Encoding]::UTF8.GetBytes($Value)
    $sha = [Security.Cryptography.SHA256]::Create()
    $hash = $sha.ComputeHash($bytes)
    return (($hash | ForEach-Object { $_.ToString("x2") }) -join "").Substring(0,24)
}

function Parse-DateToken([string]$Token, [int]$DefaultYear) {
    $m = [regex]::Match($Token, "(?:(?<y>20\d{2})\s*[.\-/년]\s*)?(?<m>1[0-2]|0?[1-9])\s*[.\-/월]\s*(?<d>3[01]|[12]\d|0?[1-9])")
    if (-not $m.Success) { return $null }
    $year = $DefaultYear
    if ($m.Groups["y"].Success) { $year = [int]$m.Groups["y"].Value }
    try {
        return (Get-Date -Year $year -Month ([int]$m.Groups["m"].Value) -Day ([int]$m.Groups["d"].Value) -Hour 0 -Minute 0 -Second 0)
    } catch { return $null }
}

function Get-DateRange([string]$Text) {
    $today = Get-Date
    $keywords = "접수기간|지원기간|신청기간|모집기간|응모기간|서류접수|원서접수|지원서 접수|지원 기간|접수 기간|신청 기간|모집 기간|응모마감|접수마감|지원마감|신청마감|마감일"
    $snips = @()
    $km = [regex]::Matches($Text, "(?is)(?:$keywords).{0,180}")
    foreach ($m in $km) { $snips += $m.Value }
    if ($snips.Count -eq 0) { $snips += $Text.Substring(0, [Math]::Min(2200, $Text.Length)) }

    foreach ($s in $snips) {
        $dm = [regex]::Matches($s, "(?:(?:20\d{2})\s*[.\-/년]\s*)?(?:1[0-2]|0?[1-9])\s*[.\-/월]\s*(?:3[01]|[12]\d|0?[1-9])(?:\s*일)?")
        if ($dm.Count -ge 2) {
            $d1 = Parse-DateToken $dm[0].Value $today.Year
            if ($null -eq $d1) { continue }
            $d2 = Parse-DateToken $dm[1].Value $d1.Year
            if ($null -eq $d2) { continue }
            if ($d2 -lt $d1) {
                $d2 = Parse-DateToken $dm[1].Value ($d1.Year + 1)
            }
            return [pscustomobject]@{
                start = $d1.ToString("yyyy-MM-dd")
                end = $d2.ToString("yyyy-MM-dd")
                confidence = "range"
            }
        }
        if ($dm.Count -eq 1) {
            $d = Parse-DateToken $dm[0].Value $today.Year
            if ($null -ne $d) {
                if ($d -lt $today.AddMonths(-6)) { $d = $d.AddYears(1) }
                return [pscustomobject]@{
                    start = $d.ToString("yyyy-MM-dd")
                    end = $d.ToString("yyyy-MM-dd")
                    confidence = "deadline-only"
                }
            }
        }
    }
    return $null
}

function Test-UsableDateRange($DateRange) {
    if ($null -eq $DateRange) { return $false }
    try {
        $start = [datetime]::ParseExact([string]$DateRange.start, "yyyy-MM-dd", $null).Date
        $end = [datetime]::ParseExact([string]$DateRange.end, "yyyy-MM-dd", $null).Date
        if ($end -lt $start) { return $false }
        if ($end -lt (Get-Date).Date) { return $false }
        return $true
    } catch { return $false }
}

function Get-ShortSummary([string]$Text, [string]$Fallback) {
    $clean = ($Text -replace "\s+", " ").Trim()
    if ($clean.Length -gt 260) { $clean = $clean.Substring(0,260) + "…" }
    if ($clean.Length -lt 20) { return $Fallback }
    return $clean
}

function Format-DetailText([string]$Text) {
    if ([string]::IsNullOrWhiteSpace($Text)) { return "" }
    $clean = ($Text -replace "`r", "")
    $clean = $clean -replace "[\u00A0\t]+", " "
    $clean = [regex]::Replace($clean, "(?m)^[ ]+|[ ]+$", "")
    $clean = [regex]::Replace($clean, "[ ]{2,}", " ")
    $clean = [regex]::Replace($clean, "바\s*로\s*가\s*기", "바로가기")
    $clean = [regex]::Replace($clean, "([0-9]{4})\s+학년도", '$1학년도')
    $clean = [regex]::Replace($clean, "([0-9])\s+차", '$1차')
    $clean = [regex]::Replace($clean, "\(\s*([월화수목금토일])\s*\)", '($1)')
    $clean = [regex]::Replace($clean, "\s+([,.:;)\]])", '$1')
    $clean = [regex]::Replace($clean, "([\(\[])\s+", '$1')
    $clean = [regex]::Replace($clean, "(?m)^\s+$", "")
    $clean = [regex]::Replace($clean, "(\n\s*){4,}", "`n`n")
    return $clean.Trim()
}

function Get-DetailText([string]$Text, [string]$Fallback) {
    $clean = Format-DetailText $Text
    if ($clean.Length -gt 6500) { $clean = $clean.Substring(0,6500) + "`n`n..." }
    if ($clean.Length -lt 20) { return $Fallback }
    return $clean
}

function Get-CleanDetailText([string]$Html, [string]$Url, [string]$Title, [string]$Fallback) {
    $bodyHtml = $Html
    if ($Url -match "admission\.ust\.ac\.kr") {
        $m = [regex]::Match($Html, '(?is)<div class="ui bbs--view--cont"[^>]*>(?<body>.*?)<div class="box-footer">')
        if ($m.Success) { $bodyHtml = $m.Groups["body"].Value }
    } elseif ($Url -match "dankook\.ac\.kr") {
        $m = [regex]::Match($Html, '(?is)<td[^>]*class="[^"]*\br_cont\b[^"]*"[^>]*>(?<body>.*?)</td>\s*</tr>')
        if ($m.Success) { $bodyHtml = $m.Groups["body"].Value }
    } elseif ($Url -match "kari\.re\.kr") {
        $m = [regex]::Match($Html, '(?is)<div class="c">\s*(?<body>.*?)</div>\s*<ul\s+class="b"')
        if ($m.Success) { $bodyHtml = $m.Groups["body"].Value }
    }

    $text = Html-ToText $bodyHtml
    if ($Url -match "admission\.ust\.ac\.kr|dankook\.ac\.kr|kari\.re\.kr") {
        $text = [regex]::Replace($text, "(?m)^\s*(본문 바로가기|대메뉴 바로가기|하단 바로가기|목록|링크복사|미리보기|프린트|공유|닫기)\s*$", "")
        $text = [regex]::Replace($text, "(?m)^\s*(LOGIN|ENGLISH|LMS|UST|전체메뉴|공지사항|입학도우미)\s*$", "")
        $text = [regex]::Replace($text, "(?m)^\s*(function\s+fn_|/\*|\*|//-->|window\.open).*?$", "")
    }
    return Get-DetailText $text $Fallback
}

function Get-CleanCompanyName([string]$Name) {
    if ([string]::IsNullOrWhiteSpace($Name)) { return "" }
    $clean = [System.Net.WebUtility]::HtmlDecode($Name)
    $clean = $clean -replace "\\u0026", "&"
    $clean = $clean -replace "\s+", " "
    $clean = $clean.Trim(" `t`r`n[]")
    return $clean
}

function Get-SaraminCompany([string]$Html, [string]$Title) {
    $m = [regex]::Match($Html, '(?is)<meta[^>]+property=["'']og:title["''][^>]+content=["''](?<v>[^"'']+)')
    if (-not $m.Success) {
        $m = [regex]::Match($Html, '(?is)<meta[^>]+content=["''](?<v>[^"'']+)["''][^>]+property=["'']og:title["'']')
    }
    if ($m.Success) {
        $v = [System.Net.WebUtility]::HtmlDecode($m.Groups["v"].Value)
        $b = [regex]::Match($v, '^\[(?<c>[^\]]+)\]')
        if ($b.Success) { return Get-CleanCompanyName $b.Groups["c"].Value }
    }
    foreach ($field in @("company_nm","main_company_nm","simple_company_nm")) {
        $m = [regex]::Match($Html, '"' + $field + '"\s*:\s*"(?<c>(?:\\.|[^"\\])*)"')
        if ($m.Success) {
            try {
                $decoded = ConvertFrom-Json ('"' + $m.Groups["c"].Value + '"')
                $company = Get-CleanCompanyName ([string]$decoded)
                if ($company) { return $company }
            } catch {}
        }
    }
    $b = [regex]::Match($Title, '^\[(?<c>[^\]]+)\]')
    if ($b.Success) { return Get-CleanCompanyName $b.Groups["c"].Value }
    return ""
}

function Get-RecruitBucket([string]$Title, [string]$Text, [string]$DefaultCategory) {
    $haystack = ([string]$Title + " " + [string]$Text)
    if ($DefaultCategory -eq "연구실 인턴" -or $haystack -match "연구인턴|연구\s*인턴|대학원\s*인턴|연구본부\s*인턴") {
        return "연구실 인턴"
    }
    if ($DefaultCategory -eq "단국대 장학" -or $DefaultCategory -eq "장학" -or $haystack -match "국가\s*장학|교내\s*장학|교외\s*장학|근로\s*장학|생활비\s*장학|학자금|장학금") {
        return "단국대 장학"
    }
    if ($haystack -match "인턴|Intern|채용전환|채용연계|전환형|연구장학생|산학장학생") {
        return "회사 인턴"
    }
    return "회사 신입채용"
}

function Get-SourceString($Source, [string]$Name, [string]$DefaultValue) {
    if ($null -ne $Source -and $Source.PSObject.Properties.Name -contains $Name) {
        $value = [string]$Source.$Name
        if (-not [string]::IsNullOrWhiteSpace($value)) { return $value }
    }
    return $DefaultValue
}

function Get-SourceRegex($Source, [string]$Name) {
    if ($null -ne $Source -and $Source.PSObject.Properties.Name -contains $Name) {
        $value = [string]$Source.$Name
        if (-not [string]::IsNullOrWhiteSpace($value)) { return $value }
    }
    return ""
}

function Get-FilteredRoleText([string]$RoleText, $Source) {
    if ([string]::IsNullOrWhiteSpace($RoleText)) { return "" }
    $excludeLine = Get-SourceRegex $Source "excludeRoleLineRegex"
    $excludeRole = Get-SourceRegex $Source "excludeRoleRegex"
    $lines = @($RoleText -split "\r?\n" | ForEach-Object { $_.Trim() } | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
    $kept = New-Object System.Collections.ArrayList
    foreach ($line in $lines) {
        if ($excludeLine -and $line -match $excludeLine) { continue }
        if ($excludeRole -and $line -match $excludeRole) { continue }
        [void]$kept.Add($line)
    }
    return ($kept -join "`n")
}

function Get-JasoseolListings($Source) {
    $items = New-Object System.Collections.ArrayList
    $seen = @{}
    $businessTypes = Get-SourceString $Source "businessTypes" ""
    if (-not $businessTypes) {
        $sourceUrl = [string]$Source.url
        if ($sourceUrl -match "[?&]businessTypes=(?<v>[^&]+)") {
            $businessTypes = [System.Uri]::UnescapeDataString($matches["v"])
        }
    }
    $divisions = Get-SourceString $Source "divisions" "1,3"
    $perPage = [int](Get-SourceString $Source "perPage" "50")
    $maxPages = [int](Get-SourceString $Source "maxPages" "5")
    $afterEnd = (Get-Date).Date.ToString("yyyy-MM-dd")
    $headers = @{
        "User-Agent" = "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 Chrome/124 Safari/537.36"
        "Accept-Language" = "ko-KR,ko;q=0.9,en-US;q=0.7,en;q=0.6"
        "Referer" = "https://jasoseol.com/search"
    }

    for ($page = 1; $page -le $maxPages; $page++) {
        $params = [ordered]@{
            page = [string]$page
            per_page = [string]$perPage
            after_end_time = $afterEnd
        }
        if ($businessTypes) { $params["by_business_types"] = $businessTypes }
        if ($divisions) { $params["by_division"] = $divisions }
        $pairs = @()
        foreach ($key in $params.Keys) {
            $pairs += ([uri]::EscapeDataString([string]$key) + "=" + [uri]::EscapeDataString([string]$params[$key]))
        }
        $apiUrl = "https://jasoseol.com/api/v1/employment_companies?" + ($pairs -join "&")
        $resp = Invoke-WebRequest -Uri $apiUrl -UseBasicParsing -TimeoutSec 25 -Headers $headers -MaximumRedirection 5
        $parsed = $resp.Content | ConvertFrom-Json
        $data = if ($parsed -is [System.Array]) { $parsed } else { @($parsed) }
        if ($data.Count -eq 0) { break }
        foreach ($item in $data) {
            if ($null -eq $item.id -or [string]::IsNullOrWhiteSpace([string]$item.name) -or [string]::IsNullOrWhiteSpace([string]$item.title)) { continue }
            $jasoseolUrl = "https://jasoseol.com/recruit/" + [string]$item.id
            if ($seen.ContainsKey($jasoseolUrl)) { continue }
            $seen[$jasoseolUrl] = $true
            $fields = @($item.employments | ForEach-Object { [string]$_.field } | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
            $divisionTokens = @($item.employments | ForEach-Object { @($_.division) } | ForEach-Object { [string]$_ } | Where-Object { $_ } | Select-Object -Unique)
            $applyUrl = [string]$item.employment_page_url
            if ([string]::IsNullOrWhiteSpace($applyUrl)) { $applyUrl = $jasoseolUrl }
            [void]$items.Add([pscustomobject]@{
                id = [string]$item.id
                company = [string]$item.name
                title = [string]$item.title
                startTime = [string]$item.start_time
                endTime = [string]$item.end_time
                url = $applyUrl
                jasoseolUrl = $jasoseolUrl
                roleText = ($fields -join "`n")
                divisions = ($divisionTokens -join ",")
                businessSize = [string]$item.company_group.business_size
                businessType = [string]$item.company_group.business_type
            })
        }
        if ($data.Count -lt $perPage) { break }
    }
    foreach ($item in $items) { Write-Output $item }
}

function Convert-UnixMsToDate([string]$Ms) {
    if ([string]::IsNullOrWhiteSpace($Ms)) { return "" }
    try {
        return ([DateTimeOffset]::FromUnixTimeMilliseconds([int64]$Ms).ToLocalTime()).ToString("yyyy-MM-dd")
    } catch { return "" }
}

function Get-LinkareerState([string]$Url) {
    $html = Get-Html $Url
    $m = [regex]::Match($html, '(?is)<script[^>]+id="__NEXT_DATA__"[^>]*>(?<json>.*?)</script>')
    if (-not $m.Success) { return $null }
    $jsonText = [System.Net.WebUtility]::HtmlDecode($m.Groups["json"].Value)
    $doc = $jsonText | ConvertFrom-Json
    return $doc.props.pageProps.__APOLLO_STATE__
}

function Get-LinkareerActivities($Source) {
    $items = New-Object System.Collections.ArrayList
    $seen = @{}
    $maxListings = [int](Get-SourceString $Source "maxListings" "120")
    $queries = @()
    if ($Source.PSObject.Properties.Name -contains "queries") { $queries = @($Source.queries) }
    if ($queries.Count -eq 0) { $queries = @("") }
    foreach ($q in $queries) {
        if ($items.Count -ge $maxListings) { break }
        $encoded = [uri]::EscapeDataString([string]$q)
        $url = "https://linkareer.com/list/recruit?filterBy_activityTypeID=5&filterBy_jobTypes=INTERN&filterBy_jobTypes=NEW&filterBy_q=$encoded&filterBy_status=OPEN&orderBy_direction=DESC&orderBy_field=RECENT&page=1"
        $state = Get-LinkareerState $url
        if ($null -eq $state) { continue }
        foreach ($key in @($state.PSObject.Properties.Name | Where-Object { $_ -match "^Activity:" })) {
            $a = $state.$key
            if ($null -eq $a.id -or [string]::IsNullOrWhiteSpace([string]$a.title)) { continue }
            $activityUrl = "https://linkareer.com/activity/" + [string]$a.id
            if ($seen.ContainsKey($activityUrl)) { continue }
            $seen[$activityUrl] = $true
            [void]$items.Add([pscustomobject]@{
                id = [string]$a.id
                company = [string]$a.organizationName
                title = [string]$a.title
                start = Convert-UnixMsToDate ([string]$a.recruitStartAt)
                end = Convert-UnixMsToDate ([string]$a.recruitCloseAt)
                url = $activityUrl
            })
            if ($items.Count -ge $maxListings) { break }
        }
    }
    foreach ($item in $items) { Write-Output $item }
}

function Get-LinkareerDetail([string]$Url, [string]$Id) {
    $state = Get-LinkareerState $Url
    if ($null -eq $state) {
        return [pscustomobject]@{ applyUrl = $Url; organizationType = ""; categories = ""; text = "" }
    }
    $activityKey = "Activity:" + $Id
    $activity = $state.$activityKey
    $applyUrl = ""
    $orgType = ""
    $categories = @()
    $detailText = ""
    if ($null -ne $activity) {
        $applyUrl = [string]$activity.applyDetail
        $orgType = [string]$activity.organizationType
        foreach ($ref in @($activity.rootCategories)) {
            $refKey = [string]$ref.__ref
            if ($refKey -and $state.PSObject.Properties.Name -contains $refKey) {
                $categories += [string]$state.$refKey.name
            }
        }
        $textRef = [string]$activity.detailText.__ref
        if ($textRef -and $state.PSObject.Properties.Name -contains $textRef) {
            $detailText = Html-ToText ([string]$state.$textRef.text)
        }
    }
    if ([string]::IsNullOrWhiteSpace($applyUrl)) { $applyUrl = $Url }
    return [pscustomobject]@{
        applyUrl = $applyUrl
        organizationType = $orgType
        categories = ($categories | Select-Object -Unique) -join ", "
        text = $detailText
    }
}

function Get-DateRangeWithFallback([string]$DetailText, [string]$Title) {
    $dr = Get-DateRange $DetailText
    if ($null -ne $dr -and (Test-UsableDateRange $dr)) { return $dr }
    $haystack = $Title + " " + $DetailText
    $m = [regex]::Match($haystack, "D-(?<d>\d+)")
    if ($m.Success) {
        $end = (Get-Date).Date.AddDays([int]$m.Groups["d"].Value)
        return [pscustomobject]@{
            start = $end.ToString("yyyy-MM-dd")
            end = $end.ToString("yyyy-MM-dd")
            confidence = "d-day"
        }
    }
    if ($haystack -match "오늘\s*마감|D-day|D0|D-0") {
        $end = (Get-Date).Date
        return [pscustomobject]@{
            start = $end.ToString("yyyy-MM-dd")
            end = $end.ToString("yyyy-MM-dd")
            confidence = "d-day"
        }
    }
    return $null
}

function Send-Balloon([string]$Title, [string]$Body) {
    try {
        Add-Type -AssemblyName System.Windows.Forms
        Add-Type -AssemblyName System.Drawing
        $n = New-Object System.Windows.Forms.NotifyIcon
        $n.Icon = [System.Drawing.SystemIcons]::Information
        $n.Visible = $true
        $n.BalloonTipIcon = [System.Windows.Forms.ToolTipIcon]::Info
        $n.BalloonTipTitle = $Title
        $n.BalloonTipText = $Body
        $n.add_BalloonTipClicked({
            Start-Process "$env:WINDIR\System32\mshta.exe" -ArgumentList ('"' + $CalendarPath + '"')
        })
        $n.ShowBalloonTip(12000)
        Start-Sleep -Seconds 8
        $n.Dispose()
    } catch {
        Log ("Notification error: " + $_.Exception.Message)
    }
}

function Upsert-Event($List, $Event) {
    $existing = $null
    foreach ($e in @($List)) {
        if ($e.id -eq $Event.id -or $e.url -eq $Event.url) { $existing = $e; break }
    }
    if ($null -ne $existing) {
        $existing.title = $Event.title
        $existing.category = $Event.category
        $existing.source = $Event.source
        if ($Event.start) { $existing.start = $Event.start }
        if ($Event.end) { $existing.end = $Event.end }
        if ($Event.PSObject.Properties.Name -contains "company") {
            Add-Member -InputObject $existing -NotePropertyName "company" -NotePropertyValue $Event.company -Force
        }
        $existing.summary = $Event.summary
        if ($Event.PSObject.Properties.Name -contains "details") {
            Add-Member -InputObject $existing -NotePropertyName "details" -NotePropertyValue $Event.details -Force
        }
        $existing.lastSeen = (Get-Date).ToString("o")
        return $false
    } else {
        $script:events += $Event
        return $true
    }
}

function Invoke-Main {
$settings = Read-JsonFile $SettingsPath ([pscustomobject]@{pollMinutes=120;notifications=$true;deadlineReminderDays=@(7,3,1,0);maxDetailFetchPerSource=18})
$sources = @(Read-JsonFile $SourcesPath @())
$script:events = @(Read-JsonFile $EventsPath @())
$seen = @(Read-JsonFile $SeenPath @())
$notified = Read-JsonFile $NotifiedPath ([pscustomobject]@{})
$seenSet = @{}
foreach ($s in $seen) { if ($s) { $seenSet[[string]$s] = $true } }
$sourceStatuses = @()
$newEvents = @()

foreach ($src in $sources) {
    $added = 0
    $checked = 0
    $err = $null
    try {
        Log ("Source start [" + $src.name + "]")
        if ($src.type -eq "html") {
            $html = Get-Html $src.url
            $anchors = @(Get-Anchors $html $src.url)
            $candidates = @()
            foreach ($a in $anchors) {
                if ($a.title -match $src.include -and (-not $src.exclude -or $a.title -notmatch $src.exclude)) {
                    if ($a.url -notmatch "^javascript:" -and $a.url -notmatch "#$") {
                        $candidates += $a
                    }
                }
            }
            $unique = @{}
            $filtered = @()
            foreach ($a in $candidates) {
                if (-not $unique.ContainsKey($a.url)) { $unique[$a.url] = $true; $filtered += $a }
            }
            $limit = [int]$settings.maxDetailFetchPerSource
            foreach ($a in @($filtered | Select-Object -First $limit)) {
                $checked++
                $detailText = ""
                try {
                    $detailHtml = Get-Html $a.url
                    $detailText = Get-CleanDetailText $detailHtml $a.url $a.title $src.summary
                } catch { $detailText = $a.title }
                $dr = Get-DateRange $detailText
                if ($null -eq $dr) { continue }
                if (-not (Test-UsableDateRange $dr)) { continue }
                $id = New-Id $a.url
                $sourceCompany = ""
                if ($src.PSObject.Properties.Name -contains "company") { $sourceCompany = [string]$src.company }
                $eventCategory = Get-RecruitBucket $a.title $detailText $src.category
                $ev = [pscustomobject]@{
                    id = $id
                    title = $a.title
                    category = $eventCategory
                    source = $src.name
                    company = $sourceCompany
                    start = $dr.start
                    end = $dr.end
                    url = $a.url
                    summary = Get-ShortSummary $detailText $src.summary
                    details = Get-DetailText $detailText $src.summary
                    detected = $dr.confidence
                    lastSeen = (Get-Date).ToString("o")
                }
                $isNew = Upsert-Event $script:events $ev
                if ($isNew) { $newEvents += $ev; $added++ }
                $seenSet[$a.url] = $true
            }
        }
        elseif ($src.type -eq "kai_jobflex") {
            $api = Invoke-KaiJobflexApi
            foreach ($item in @($api.list)) {
                $checked++
                $title = Repair-Mojibake ([string]$item.title)
                if ([string]::IsNullOrWhiteSpace($title)) { continue }
                if ([string]$item.submissionStatus -ne "IN_SUBMISSION") { continue }
                if ($src.include -and $title -notmatch $src.include -and [string]$item.careerType -ne "NEW") { continue }
                if ($src.exclude -and $title -match $src.exclude) { continue }
                if (-not $item.endDateTime) { continue }

                $dr = [pscustomobject]@{
                    start = ([datetime]::Parse([string]$item.startDateTime)).ToString("yyyy-MM-dd")
                    end = ([datetime]::Parse([string]$item.endDateTime)).ToString("yyyy-MM-dd")
                    confidence = "kai-api"
                }
                if (-not (Test-UsableDateRange $dr)) { continue }

                $url = "https://koreaaero.recruiter.co.kr/career/job/" + [string]$item.positionSn
                $id = New-Id $url
                $summary = "KAI 공식 채용 사이트에서 접수 중인 공고. 접수/모집: {0} ~ {1}" -f $dr.start, $dr.end
                $eventCategory = Get-RecruitBucket $title $summary $src.category
                $ev = [pscustomobject]@{
                    id = $id
                    title = $title
                    category = $eventCategory
                    source = $src.name
                    company = "KAI"
                    start = $dr.start
                    end = $dr.end
                    url = $url
                    summary = $summary
                    details = $summary
                    detected = $dr.confidence
                    lastSeen = (Get-Date).ToString("o")
                }
                $isNew = Upsert-Event $script:events $ev
                if ($isNew) { $newEvents += $ev; $added++ }
                $seenSet[$url] = $true
            }
        }
        elseif ($src.type -eq "jasoseol_html") {
            $listings = @(Get-JasoseolListings $src)
            $limit = [int]$settings.maxDetailFetchPerSource
            if ($src.PSObject.Properties.Name -contains "maxListings") { $limit = [int]$src.maxListings }
            foreach ($item in @($listings | Select-Object -First $limit)) {
                $checked++
                try {
                    if ([string]::IsNullOrWhiteSpace([string]$item.endTime)) { continue }
                    $dr = [pscustomobject]@{
                        start = ([datetimeoffset]::Parse([string]$item.startTime)).ToString("yyyy-MM-dd")
                        end = ([datetimeoffset]::Parse([string]$item.endTime)).ToString("yyyy-MM-dd")
                        confidence = "jasoseol"
                    }
                    if (-not (Test-UsableDateRange $dr)) { continue }
                    $title = [string]$item.title
                    $company = [string]$item.company
                    $rawRoleText = [string]$item.roleText
                    $roleText = Get-FilteredRoleText $rawRoleText $src
                    if (-not [string]::IsNullOrWhiteSpace($rawRoleText) -and [string]::IsNullOrWhiteSpace($roleText)) { continue }
                    $combined = $company + " " + $title + " " + $roleText
                    $requireCompanyName = ($src.PSObject.Properties.Name -contains "requireCompanyNameRegex" -and [bool]$src.requireCompanyNameRegex)
                    if ($requireCompanyName -and $src.companyNameRegex -and $company -notmatch [string]$src.companyNameRegex) { continue }
                    if ($src.PSObject.Properties.Name -contains "hardExcludeRegex" -and $combined -match [string]$src.hardExcludeRegex) { continue }
                    if ($src.companyNameRegex -and $company -notmatch [string]$src.companyNameRegex -and $combined -notmatch [string]$src.majorRegex) { continue }
                    if ($src.excludeRoleRegex -and $combined -match $src.excludeRoleRegex -and $combined -notmatch $src.majorRegex) { continue }
                    if ($combined -notmatch $src.majorRegex) { continue }
                    if ($combined -notmatch $src.careerRegex) { continue }
                    $detailText = "모집 직무`n" + $roleText
                    $eventCategory = Get-RecruitBucket $title ($roleText + " " + $detailText) $src.category
                    $id = New-Id ([string]$item.jasoseolUrl)
                    $periodText = if ($dr.start -eq $dr.end) { "마감일: " + $dr.end } else { "접수/모집: " + $dr.start + " ~ " + $dr.end }
                    $roleBlock = if ([string]::IsNullOrWhiteSpace($roleText)) { "" } else { "`n`n모집 직무:`n- " + ($roleText -replace "`n", "`n- ") }
                    $details = "자소설닷컴 채용 API에서 찾은 대기업/공공기관 전공 관련 공고입니다.`n`n회사: " + $company + "`n공고: " + $title + $roleBlock + "`n`n" + $periodText + "`n지원 링크: " + [string]$item.url + "`n자소설 링크: " + [string]$item.jasoseolUrl + "`n`n" + (Get-DetailText $detailText $src.summary)
                    $ev = [pscustomobject]@{
                        id = $id
                        title = $title
                        category = $eventCategory
                        source = $src.name
                        company = $company
                        start = $dr.start
                        end = $dr.end
                        url = [string]$item.url
                        summary = Get-ShortSummary $details $src.summary
                        details = $details
                        detected = $dr.confidence
                        lastSeen = (Get-Date).ToString("o")
                    }
                    $isNew = Upsert-Event $script:events $ev
                    if ($isNew) { $newEvents += $ev; $added++ }
                    $seenSet[[string]$item.jasoseolUrl] = $true
                    $seenSet[[string]$item.url] = $true
                } catch { Log ("Jasoseol detail failure: " + [string]$item.jasoseolUrl + " / " + $_.Exception.Message) }
            }
        }
        elseif ($src.type -eq "linkareer_html") {
            $listings = @(Get-LinkareerActivities $src)
            $limit = [int]$settings.maxDetailFetchPerSource
            if ($src.PSObject.Properties.Name -contains "maxListings") { $limit = [int]$src.maxListings }
            foreach ($item in @($listings | Select-Object -First $limit)) {
                $checked++
                try {
                    if ([string]::IsNullOrWhiteSpace([string]$item.end)) { continue }
                    $dr = [pscustomobject]@{
                        start = if ([string]::IsNullOrWhiteSpace([string]$item.start)) { [string]$item.end } else { [string]$item.start }
                        end = [string]$item.end
                        confidence = "linkareer"
                    }
                    if (-not (Test-UsableDateRange $dr)) { continue }
                    $title = [string]$item.title
                    $company = [string]$item.company
                    $requireCompanyName = ($src.PSObject.Properties.Name -contains "requireCompanyNameRegex" -and [bool]$src.requireCompanyNameRegex)
                    if ($requireCompanyName -and $src.companyNameRegex -and $company -notmatch [string]$src.companyNameRegex) { continue }
                    if (-not $requireCompanyName -and $src.companyNameRegex -and ($company + " " + $title) -notmatch [string]$src.companyNameRegex) { continue }
                    $preCombined = $company + " " + $title
                    if ($src.PSObject.Properties.Name -contains "hardExcludeRegex" -and $preCombined -match [string]$src.hardExcludeRegex) { continue }
                    if ($src.excludeRoleRegex -and $preCombined -match [string]$src.excludeRoleRegex -and $preCombined -notmatch [string]$src.majorRegex) { continue }
                    $detail = Get-LinkareerDetail ([string]$item.url) ([string]$item.id)
                    $combined = $company + " " + $title + " " + [string]$detail.organizationType + " " + [string]$detail.categories + " " + [string]$detail.text
                    if ($src.PSObject.Properties.Name -contains "hardExcludeRegex" -and $combined -match [string]$src.hardExcludeRegex) { continue }
                    if ($src.companySizeRegex -and [string]$detail.organizationType -notmatch [string]$src.companySizeRegex -and ($company + " " + $title) -notmatch [string]$src.companyNameRegex) { continue }
                    if ($src.excludeRoleRegex -and $combined -match $src.excludeRoleRegex -and $combined -notmatch $src.majorRegex) { continue }
                    if ($combined -notmatch $src.majorRegex) { continue }
                    if ($combined -notmatch $src.careerRegex) { continue }
                    $eventCategory = Get-RecruitBucket $title $combined $src.category
                    $id = New-Id ([string]$item.url)
                    $periodText = if ($dr.start -eq $dr.end) { "마감일: " + $dr.end } else { "접수/모집: " + $dr.start + " ~ " + $dr.end }
                    $details = "링커리어에서 찾은 대기업 전공 관련 공고입니다.`n`n회사: " + $company + "`n공고: " + $title + "`n기업형태: " + [string]$detail.organizationType + "`n모집직무: " + [string]$detail.categories + "`n`n" + $periodText + "`n지원 링크: " + [string]$detail.applyUrl + "`n링커리어 링크: " + [string]$item.url + "`n`n" + (Get-DetailText ([string]$detail.text) $src.summary)
                    $ev = [pscustomobject]@{
                        id = $id
                        title = $title
                        category = $eventCategory
                        source = $src.name
                        company = $company
                        start = $dr.start
                        end = $dr.end
                        url = [string]$detail.applyUrl
                        summary = Get-ShortSummary $details $src.summary
                        details = $details
                        detected = $dr.confidence
                        lastSeen = (Get-Date).ToString("o")
                    }
                    $isNew = Upsert-Event $script:events $ev
                    if ($isNew) { $newEvents += $ev; $added++ }
                    $seenSet[[string]$item.url] = $true
                    $seenSet[[string]$detail.applyUrl] = $true
                } catch { Log ("Linkareer detail failure: " + [string]$item.url + " / " + $_.Exception.Message) }
            }
        }
        elseif ($src.type -eq "saramin_html") {
            $resultLinks = New-Object System.Collections.ArrayList
            $resultSeen = @{}
            $perQueryLimit = 0
            if ($src.PSObject.Properties.Name -contains "maxResultsPerQuery") { $perQueryLimit = [int]$src.maxResultsPerQuery }
            $queryList = @($src.queries)
            if ($src.PSObject.Properties.Name -contains "maxSearchQueries") {
                $queryList = @($queryList | Select-Object -First ([int]$src.maxSearchQueries))
            }
            foreach ($q in $queryList) {
                $encoded = [uri]::EscapeDataString([string]$q)
                $u = "https://www.saramin.co.kr/zf_user/search/recruit?searchword=$encoded"
                try {
                    $html = Get-Html $u
                    $anchors = @(Get-Anchors $html $u)
                    $addedForQuery = 0
                    foreach ($a in $anchors) {
                        if ($perQueryLimit -gt 0 -and $addedForQuery -ge $perQueryLimit) { break }
                        if ($a.title -match "홈페이지\s*지원|입사지원|스크랩") { continue }
                        if ($a.url -match "/zf_user/jobs/(?:relay/)?view\?.*rec_idx=(?<rec>\d+)") {
                            $cleanUrl = "https://www.saramin.co.kr/zf_user/jobs/relay/view?rec_idx=" + $matches["rec"]
                            if (-not $resultSeen.ContainsKey($cleanUrl)) {
                                $resultSeen[$cleanUrl] = $true
                                [void]$resultLinks.Add([pscustomobject]@{ url = $cleanUrl; title = $a.title })
                                $addedForQuery++
                            }
                        }
                    }
                } catch { Log ("Saramin search failure: " + $q + " / " + $_.Exception.Message) }
            }
            $limit = [int]$settings.maxDetailFetchPerSource
            foreach ($result in @($resultLinks | Select-Object -First $limit)) {
                $checked++
                try {
                    $url = [string]$result.url
                    $detailHtml = Get-Html $url
                    $detailText = Html-ToText $detailHtml
                    $title = [string]$result.title
                    $company = Get-SaraminCompany $detailHtml $title
                    if ([string]::IsNullOrWhiteSpace($company)) { continue }
                    $companyProof = $company + " " + $title
                    $hasLargeProof = $false
                    if ($src.companyNameRegex -and $company -match [string]$src.companyNameRegex) { $hasLargeProof = $true }
                    if ($src.companySizeRegex -and $company -match [string]$src.companySizeRegex) { $hasLargeProof = $true }
                    if (-not $hasLargeProof) { continue }
                    $combined = $company + " " + $title + " " + $detailText
                    $majorLike = $combined -match $src.majorRegex
                    $broadRecruit = $title -match "신입|공채|공개채용|채용전환|채용연계|인턴|대졸"
                    if (-not $majorLike -and -not $broadRecruit) { continue }
                    if ($combined -notmatch $src.careerRegex) { continue }
                    if ($src.excludeRoleRegex -and $title -match $src.excludeRoleRegex) { continue }
                    if ($src.excludeCompanySizeRegex -and ($company + " " + $title) -match $src.excludeCompanySizeRegex) { continue }
                    if ($combined -match $src.excludeCompanySizeRegex -and $combined -notmatch "대기업|중견기업|코스피|코스닥|코넥스|공기업|공공기관") { continue }
                    $dr = Get-DateRangeWithFallback $detailText $title
                    if ($null -eq $dr) { continue }
                    if (-not (Test-UsableDateRange $dr)) { continue }
                    $id = New-Id $url
                    $periodText = if ($dr.start -eq $dr.end) { "마감일: " + $dr.end } else { "접수/모집: " + $dr.start + " ~ " + $dr.end }
                    $saraminDetails = "사람인 검색에서 찾은 대기업급/공기업급 전자·전기·제어 직무 공고입니다.`n`n회사: " + $company + "`n공고: " + $title + "`n`n" + $periodText + "`n원문/지원: " + $url
                    $eventCategory = Get-RecruitBucket $title $detailText $src.category
                    $ev = [pscustomobject]@{
                        id = $id
                        title = $title
                        category = $eventCategory
                        source = $src.name
                        company = $company
                        start = $dr.start
                        end = $dr.end
                        url = $url
                        summary = Get-ShortSummary $saraminDetails $src.summary
                        details = $saraminDetails
                        detected = $dr.confidence
                        lastSeen = (Get-Date).ToString("o")
                    }
                    $isNew = Upsert-Event $script:events $ev
                    if ($isNew) { $newEvents += $ev; $added++ }
                    $seenSet[$url] = $true
                } catch { Log ("Saramin detail failure: " + $url + " / " + $_.Exception.Message) }
            }
        }
        $sourceStatuses += [pscustomobject]@{
            name = $src.name
            ok = $true
            checked = $checked
            added = $added
            checkedAt = (Get-Date).ToString("o")
            error = ""
        }
    } catch {
        $err = $_.Exception.Message
        Log ("Source failure [" + $src.name + "]: " + $err)
        $sourceStatuses += [pscustomobject]@{
            name = $src.name
            ok = $false
            checked = $checked
            added = $added
            checkedAt = (Get-Date).ToString("o")
            error = $err
        }
    }
}

# Keep only sane event objects, apply current source excludes, and sort by start date.
$sourceByName = @{}
foreach ($src in $sources) { $sourceByName[[string]$src.name] = $src }
$saraminCfg = @($sources | Where-Object { $_.type -eq "saramin_html" } | Select-Object -First 1)
$script:events = @($script:events | Where-Object {
    $keep = $_.title -and $_.start -and $_.end -and $_.url
    if ($keep -and (-not ($_.PSObject.Properties.Name -contains "company") -or [string]::IsNullOrWhiteSpace([string]$_.company))) {
        $defaultCompany = ""
        if ([string]$_.category -eq "KAI") { $defaultCompany = "KAI" }
        elseif ([string]$_.category -eq "KARI") { $defaultCompany = "KARI" }
        elseif ([string]$_.category -eq "UST") { $defaultCompany = "UST" }
        elseif ([string]$_.category -eq "DGIST") { $defaultCompany = "DGIST" }
        elseif ([string]$_.category -eq "장학") { $defaultCompany = "단국대" }
        if ($defaultCompany) { Add-Member -InputObject $_ -NotePropertyName "company" -NotePropertyValue $defaultCompany -Force }
    }
    if ($keep) {
        $oldCategory = [string]$_.category
        if ($oldCategory -match "^(KAI|KARI|전자전기)$") {
            $_.category = Get-RecruitBucket $_.title ([string]$_.summary + " " + [string]$_.details) "회사 신입채용"
        } elseif ($oldCategory -match "^(UST|DGIST)$") {
            $_.category = "연구실 인턴"
        } elseif ($oldCategory -eq "장학") {
            $_.category = "단국대 장학"
        } elseif ($oldCategory -match "^(회사 신입채용|회사 인턴)$") {
            $_.category = Get-RecruitBucket $_.title ([string]$_.summary + " " + [string]$_.details) $oldCategory
        }
    }
    if ($keep -and ($sourceByName.ContainsKey([string]$_.source) -or ([string]$_.category -eq "전자전기" -and $saraminCfg))) {
        $cfg = if ($sourceByName.ContainsKey([string]$_.source)) { $sourceByName[[string]$_.source] } else { $saraminCfg }
        if ($cfg.exclude -and [string]$_.title -match [string]$cfg.exclude) { $keep = $false }
        if ($cfg.excludeRoleRegex -and [string]$_.title -match [string]$cfg.excludeRoleRegex -and [string]$_.title -notmatch [string]$cfg.majorRegex) { $keep = $false }
        if ($cfg.excludeCompanySizeRegex -and [string]$_.title -match [string]$cfg.excludeCompanySizeRegex) { $keep = $false }
        if ($cfg.type -eq "jasoseol_html") {
            $combined = ([string]$_.company + " " + [string]$_.title + " " + [string]$_.summary + " " + [string]$_.details)
            if ($keep -and $cfg.PSObject.Properties.Name -contains "hardExcludeRegex" -and $combined -match [string]$cfg.hardExcludeRegex) { $keep = $false }
            $requireCompanyName = ($cfg.PSObject.Properties.Name -contains "requireCompanyNameRegex" -and [bool]$cfg.requireCompanyNameRegex)
            if ($keep -and $requireCompanyName -and $cfg.companyNameRegex -and [string]$_.company -notmatch [string]$cfg.companyNameRegex) { $keep = $false }
            if ($keep -and $cfg.majorRegex -and $combined -notmatch [string]$cfg.majorRegex) { $keep = $false }
            if ($keep -and $cfg.careerRegex -and $combined -notmatch [string]$cfg.careerRegex) { $keep = $false }
            if ($keep -and $cfg.excludeRoleRegex -and $combined -match [string]$cfg.excludeRoleRegex -and $combined -notmatch [string]$cfg.majorRegex) { $keep = $false }
            if ($keep -and $cfg.companyNameRegex -and [string]$_.company -notmatch [string]$cfg.companyNameRegex -and $combined -notmatch [string]$cfg.majorRegex) { $keep = $false }
        }
        if ($cfg.type -eq "linkareer_html") {
            $combined = ([string]$_.company + " " + [string]$_.title + " " + [string]$_.summary + " " + [string]$_.details)
            $requireCompanyName = ($cfg.PSObject.Properties.Name -contains "requireCompanyNameRegex" -and [bool]$cfg.requireCompanyNameRegex)
            if ($keep -and $cfg.PSObject.Properties.Name -contains "hardExcludeRegex" -and $combined -match [string]$cfg.hardExcludeRegex) { $keep = $false }
            if ($keep -and $requireCompanyName -and $cfg.companyNameRegex -and [string]$_.company -notmatch [string]$cfg.companyNameRegex) { $keep = $false }
            if ($keep -and -not $requireCompanyName -and $cfg.companyNameRegex -and ([string]$_.company + " " + [string]$_.title) -notmatch [string]$cfg.companyNameRegex) { $keep = $false }
            if ($keep -and $cfg.majorRegex -and $combined -notmatch [string]$cfg.majorRegex) { $keep = $false }
            if ($keep -and $cfg.careerRegex -and $combined -notmatch [string]$cfg.careerRegex) { $keep = $false }
            if ($keep -and $cfg.excludeRoleRegex -and $combined -match [string]$cfg.excludeRoleRegex -and $combined -notmatch [string]$cfg.majorRegex) { $keep = $false }
        }
        if ($cfg.type -eq "saramin_html") {
            $combined = ([string]$_.company + " " + [string]$_.title + " " + [string]$_.summary + " " + [string]$_.details)
            $companyProof = ([string]$_.company + " " + [string]$_.title)
            $hasLargeProof = $false
            if ($cfg.companyNameRegex -and [string]$_.company -match [string]$cfg.companyNameRegex) { $hasLargeProof = $true }
            if ($cfg.companySizeRegex -and [string]$_.company -match [string]$cfg.companySizeRegex) { $hasLargeProof = $true }
            if ([string]::IsNullOrWhiteSpace([string]$_.company)) { $keep = $false }
            if ($keep -and -not $hasLargeProof) { $keep = $false }
            if ($keep -and $cfg.excludeRoleRegex -and [string]$_.title -match [string]$cfg.excludeRoleRegex) { $keep = $false }
            if ($keep -and $cfg.excludeCompanySizeRegex -and ([string]$_.company + " " + [string]$_.title) -match [string]$cfg.excludeCompanySizeRegex) { $keep = $false }
        }
    }
    $keep
} | Sort-Object start, end, title)
Write-JsonFile $EventsPath $script:events
Write-JsonFile $SeenPath @($seenSet.Keys)

# Notifications for newly found postings.
if ($settings.notifications -and -not $Silent) {
    $pendingNew = @()
    foreach ($ev in $newEvents) {
        $k = "new_" + $ev.id
        if (-not ($notified.PSObject.Properties.Name -contains $k)) {
            $pendingNew += $ev
        }
    }
    if ($pendingNew.Count -eq 1) {
        $ev = $pendingNew[0]
        $body = ("{0}`n접수/모집: {1} ~ {2}" -f $ev.title, $ev.start, $ev.end)
        Send-Balloon "새 모집공고" $body
    } elseif ($pendingNew.Count -gt 1) {
        $lines = @($pendingNew | Select-Object -First 5 | ForEach-Object { "- " + $_.title })
        if ($pendingNew.Count -gt 5) { $lines += ("외 {0}건" -f ($pendingNew.Count - 5)) }
        Send-Balloon ("새 모집공고 {0}건" -f $pendingNew.Count) ($lines -join "`n")
    }
    foreach ($ev in $pendingNew) {
        $k = "new_" + $ev.id
        Add-Member -InputObject $notified -NotePropertyName $k -NotePropertyValue (Get-Date).ToString("o") -Force
    }

    # Deadline reminders.
    $today0 = (Get-Date).Date
    $daysWanted = @($settings.deadlineReminderDays | ForEach-Object { [int]$_ })
    foreach ($ev in $script:events) {
        try {
            $end = [datetime]::ParseExact([string]$ev.end, "yyyy-MM-dd", $null).Date
            $left = [int]($end - $today0).TotalDays
            if ($daysWanted -contains $left) {
                $k = "d" + $left + "_" + $ev.id
                if (-not ($notified.PSObject.Properties.Name -contains $k)) {
                    $label = if ($left -eq 0) { "오늘 마감" } else { "마감 D-" + $left }
                    Send-Balloon $label ($ev.title + "`n" + $ev.end + " 마감")
                    Add-Member -InputObject $notified -NotePropertyName $k -NotePropertyValue (Get-Date).ToString("o") -Force
                }
            }
        } catch {}
    }
}
Write-JsonFile $NotifiedPath $notified

$status = [pscustomobject]@{
    lastUpdate = (Get-Date).ToString("o")
    sources = $sourceStatuses
    message = ("확인 완료: 새 공고 {0}건" -f $newEvents.Count)
}
Write-JsonFile $StatusPath $status
Log $status.message
}

$updateMutexCreated = $false
$updateMutex = New-Object System.Threading.Mutex($true, "Local\RecruitCalendarUpdater_v2", [ref]$updateMutexCreated)
if (-not $updateMutexCreated) {
    Log "확인 건너뜀: 이미 다른 업데이트가 실행 중"
    exit 0
}

try {
    Invoke-Main
}
finally {
    if ($updateMutex) { $updateMutex.ReleaseMutex(); $updateMutex.Dispose() }
}
