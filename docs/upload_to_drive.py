#!/usr/bin/env python3
"""
IVS_Final → Google Drive 자동 업로더
=====================================
최초 1회 준비 (2분):
  1. https://console.cloud.google.com/apis/dashboard 접속
  2. 프로젝트 생성 (또는 기존 프로젝트 선택)
  3. "API 및 서비스" > "라이브러리" > "Google Drive API" > 사용 설정
  4. "API 및 서비스" > "사용자 인증 정보" > "+ 사용자 인증 정보 만들기" > "OAuth 클라이언트 ID"
     - 애플리케이션 유형: 데스크톱 앱
     - 이름: 아무거나
  5. JSON 다운로드 → 이 스크립트 옆(docs/ 폴더)에 credentials.json 으로 저장
  6. py docs/upload_to_drive.py  (처음 실행 시 브라우저 열림 → Google 로그인 → 허용 클릭)

이후 실행: py docs/upload_to_drive.py  (바로 업로드 시작)
"""

import os
import sys
import webbrowser
from pathlib import Path

DOCS_ROOT = Path(__file__).parent
CREDS_FILE = DOCS_ROOT / "credentials.json"
TOKEN_FILE = DOCS_ROOT / "token.json"

# ── Google Drive 폴더 ID (이미 생성 완료) ────────────────────────────────────
FOLDER_IDS = {
    "":                            "1WH6_Qx0AJbrZjKgbX95EsB-EVvRzKvIJ",
    "01_SYS.1_고객요구사항":       "1VlzfmW_kLLPlJgt0SMEHF6TYTbye26Tr",
    "02_SYS.2_시스템요구사항":     "1eXRprwXrmlM-7K-pVZ9alpL25yykwjMX",
    "03_SYS.3_시스템아키텍처":     "1WQV1wp9sWHPPBrDVy25z8w9fg7n3a9qe",
    "04_SWE.1_SW요구사항":         "1q0af0zWFhmV-psYwwG6uuoUU0EJFP8LK",
    "05_SWE.2_SW아키텍처":         "1Rj5LXfonDsVqPYBBlui7akNJlBIo5kS2",
    "06_SWE.3_SW상세설계":         "1Z23o-GnqIgG1QFh5qBfmKV0BHzLmVK_h",
    "07_HWE.1_HW요구사항":         "1GPe5Ccn1zaGXs4Cm_czsyfjOv190wt6K",
    "08_HWE.2_HW설계":             "1rD9TWtNrkoY7UAYQRwmZtZH32qAJLQtm",
    "09_SWE.4-5_SW단위통합검증":   "1rwdzncz8Vm58G7uEjvwEj9nlCv3WBhRG",
    "10_SWE.6_SW검증":             "1ogiSg7MGezdup2YQiRtWfRiSV4pLg30d",
    "11_HWE.3-4_HW검증":           "1K0eds0CkaseAgvQEZbhs0y4odiDN_O_-",
    "12_SYS.4_시스템통합검증":     "1v6ADMsdVuYofOmrx6yEM-_eMomP4Wh3d",
    "13_SYS.5_시스템검증":         "1QPA7OV2jp01c6TwtJFMsex9Qo1wl54Ep",
    "14_VAL.1_시스템Validation":   "1DUvhZdZy38GkEwTMMu6Cqu9hLCnuysJ7",
    "old":                         "1v2lw0-w7yQd7OvKAEV1XXwIyU8zpsROB",
}

FILES = [
    ("주행로봇을 활용한 자율주행 프로젝트 실습자료 ver 1.8.pdf",           ""),
    ("01_SYS.1_고객요구사항/1조_군집주행_컨셉_발표자료.pptx",              "01_SYS.1_고객요구사항"),
    ("01_SYS.1_고객요구사항/1조_군집주행_컨셉_발표자료.pdf",               "01_SYS.1_고객요구사항"),
    ("02_SYS.2_시스템요구사항/SYS-PLT-001_시스템요구사항명세서_v1.1.docx", "02_SYS.2_시스템요구사항"),
    ("03_SYS.3_시스템아키텍처/SYA-PLT-001_시스템아키텍처설계서_v1.0.docx", "03_SYS.3_시스템아키텍처"),
    ("03_SYS.3_시스템아키텍처/D_system_architecture.png",                  "03_SYS.3_시스템아키텍처"),
    ("04_SWE.1_SW요구사항/SRS-PLT-001_SW요구사항명세서_v1_3.docx",         "04_SWE.1_SW요구사항"),
    ("05_SWE.2_SW아키텍처/SAD-PLT-001_SW아키텍처설계서_v1.0.docx",         "05_SWE.2_SW아키텍처"),
    ("05_SWE.2_SW아키텍처/ICD-PLT-001_인터페이스명세서_v1.0.docx",         "05_SWE.2_SW아키텍처"),
    ("05_SWE.2_SW아키텍처/SW_architecture.png",                            "05_SWE.2_SW아키텍처"),
    ("05_SWE.2_SW아키텍처/Communication_architecture.png",                 "05_SWE.2_SW아키텍처"),
    ("06_SWE.3_SW상세설계/SDD-PLT-001_SW상세설계서_v1.0.docx",             "06_SWE.3_SW상세설계"),
    ("07_HWE.1_HW요구사항/HRS-PLT-001_HW요구사항명세서_v1.0.docx",         "07_HWE.1_HW요구사항"),
    ("08_HWE.2_HW설계/HWD-PLT-001_HW설계서_v1.0.docx",                     "08_HWE.2_HW설계"),
    ("08_HWE.2_HW설계/HW_design.png",                                      "08_HWE.2_HW설계"),
    ("09_SWE.4-5_SW단위통합검증/SUT-PLT-001_SW단위통합테스트명세서_v1.0.docx", "09_SWE.4-5_SW단위통합검증"),
    ("10_SWE.6_SW검증/SWV-PLT-001_SW검증명세서_v1.0.docx",                 "10_SWE.6_SW검증"),
    ("11_HWE.3-4_HW검증/HVT-PLT-001_HW검증명세서_v1.0.docx",               "11_HWE.3-4_HW검증"),
    ("12_SYS.4_시스템통합검증/SIT-PLT-001_시스템통합검증명세서_v1.0.docx", "12_SYS.4_시스템통합검증"),
    ("13_SYS.5_시스템검증/SYV-PLT-001_시스템검증명세서_v1.0.docx",         "13_SYS.5_시스템검증"),
    ("14_VAL.1_시스템Validation/VAL-PLT-001_시스템Validation명세서_v1.0.docx", "14_VAL.1_시스템Validation"),
    ("old/D_system_architecture_구버전.png",                               "old"),
    ("old/Communication_architecture_구버전.png",                          "old"),
    ("old/MDS-PLT-001_모듈스펙명세서_v1.0.docx",                           "old"),
    ("old/SYS-PLT-001_시스템요구사항명세서_v1.0.docx",                     "old"),
    ("old/SRS-PLT-001_SW요구사항명세서_v1_2.docx",                         "old"),
]

SCOPES = ["https://www.googleapis.com/auth/drive.file"]


def get_credentials():
    from google.oauth2.credentials import Credentials
    from google_auth_oauthlib.flow import InstalledAppFlow
    from google.auth.transport.requests import Request

    creds = None
    if TOKEN_FILE.exists():
        creds = Credentials.from_authorized_user_file(str(TOKEN_FILE), SCOPES)

    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(str(CREDS_FILE), SCOPES)
            creds = flow.run_local_server(port=0)
        TOKEN_FILE.write_text(creds.to_json())

    return creds


def upload_file(service, file_path: Path, folder_id: str) -> str:
    from googleapiclient.http import MediaFileUpload

    import mimetypes
    mime, _ = mimetypes.guess_type(str(file_path))
    mime = mime or "application/octet-stream"

    meta = {"name": file_path.name, "parents": [folder_id]}
    media = MediaFileUpload(str(file_path), mimetype=mime, resumable=True)
    result = service.files().create(body=meta, media_body=media, fields="id").execute()
    return result.get("id", "?")


def main():
    if not CREDS_FILE.exists():
        print("=" * 60)
        print("credentials.json 파일이 없습니다.")
        print()
        print("아래 단계를 따라 2분 내 준비 가능합니다:")
        print()
        print("1. 브라우저가 열리면 Google Cloud Console로 이동합니다.")
        print("2. 프로젝트 선택 또는 새로 만들기")
        print("3. 'API 및 서비스' > '라이브러리' 에서 'Google Drive API' 검색 후 사용 설정")
        print("4. '사용자 인증 정보' > '+ 사용자 인증 정보 만들기' > 'OAuth 클라이언트 ID'")
        print("   - 애플리케이션 유형: 데스크톱 앱")
        print("5. JSON 다운로드 후 이 폴더에 credentials.json 으로 저장")
        print()
        print("준비 완료 후 다시 실행하세요: py docs/upload_to_drive.py")
        print("=" * 60)
        webbrowser.open("https://console.cloud.google.com/apis/credentials")
        sys.exit(0)

    try:
        from googleapiclient.discovery import build
    except ImportError:
        print("패키지 설치 중...")
        os.system(f"{sys.executable} -m pip install google-api-python-client google-auth-oauthlib -q")
        from googleapiclient.discovery import build

    print("Google 인증 중... (브라우저가 열리면 '허용' 클릭)")
    creds = get_credentials()
    service = build("drive", "v3", credentials=creds)
    print("인증 완료!\n")

    ok = skip = fail = 0
    for rel, folder_key in FILES:
        file_path = DOCS_ROOT / rel
        if not file_path.exists():
            print(f"  [SKIP] {rel}")
            skip += 1
            continue
        folder_id = FOLDER_IDS[folder_key]
        try:
            file_id = upload_file(service, file_path, folder_id)
            print(f"  [OK]   {file_path.name}")
            ok += 1
        except Exception as e:
            print(f"  [FAIL] {file_path.name} — {e}")
            fail += 1

    print(f"\n완료: 성공 {ok} / 스킵 {skip} / 실패 {fail}")
    print(f"Drive 폴더: https://drive.google.com/drive/folders/1WH6_Qx0AJbrZjKgbX95EsB-EVvRzKvIJ")


if __name__ == "__main__":
    main()
