import os
import re

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate
# from langchain.chains import LLMChain

from cobot2.wakeup_word import WakeupWord
from cobot2.stt import STT


# Goal Result's code section
CODE_OK = 0
CODE_TIMEOUT = 1
CODE_MISMATCH = 2
CODE_CANCELED = 3
############ GetKeyword Node ############
class GetKeyword():
    """
    expected(정답 답어) + heard_text(거수자 발화) 를 넣으면
    verdict/score/extracted/reason 을 반환하는 "검증기" 클래스
    """
    def __init__(self, temperature: float = 0.3):
        pkg = get_package_share_directory("cobot2")
        env_path = os.path.join(pkg, "resource", ".env")
        if os.path.exists(env_path):
            load_dotenv(env_path)
        api_key = os.getenv("OPENAI_API_KEY")

        if not api_key:
            raise RuntimeError("OPENAI_API_KEY not found. (resource/.env 로드 실패 또는 환경변수 미설정)")


        # temperature : 출력의 랜덤성(확률 분포의 퍼짐 정도)
        # 0 ~ 0.3 가 정석
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=temperature, openai_api_key=api_key
        )

        prompt_content = """
        당신은 음성인식(STT) 텍스트에서 "답어"를 검증하는 심사관입니다.

        [절대 규칙: 억지 매칭 금지]
        - heard_text에 expected가 "우연히" 포함될 가능성을 항상 고려하십시오.
        - 사용자가 단순 잡담/감탄/문장 일부로 말한 것을 "답어 발화"로 강제 해석하면 안 됩니다.
        - expected를 맞추기 위해 heard_text를 재구성하거나 의미를 바꾸거나 없는 단어를 만들어내면 안 됩니다.

        [OK 판정 조건]
        1) expected가 heard_text에서 "독립적으로 발화"된 것으로 보일 것.
        - 단독 발화, 짧은 응답(조사/어미 소량), 공백 분리, 1~2글자 오탈자 정도만 허용
        2) heard_text가 '답을 말하는 상황'으로 자연스러울 것(정답은~, ~입니다 등) 또는 전체 발화가 매우 짧고 핵심이 expected일 것.
        3) 다음은 OK 금지:
        - 긴 문장 속 우연 포함
        - 잡담/감탄("아우 졸리다")처럼 의미가 다른데 글자만 비슷
        - 단어 경계를 억지로 끊거나 합쳐서 expected를 만드는 해석

        [판정]
        - OK: 의도적으로 expected를 답어로 말한 것이 강하게 확실
        - MISMATCH: 다른 답어를 말했거나 expected가 명확히 아님. 또는 애매함(억지 매칭 가능성이 있어 확정하면 위험)

        [출력 형식]
        한 줄로만 출력:
        <VERDICT> / extracted=<원문 조각 또는 빈칸> / reason=<짧게 근거>

        [입력]
        expected: "{expected}"
        heard_text: "{heard_text}"
        """
        self.prompt_template = PromptTemplate(
            input_variables=["expected", "heard_text"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm
        # self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)



    def verify(self, expected: str, heard_text: str) -> dict:
        """
        minimum format for Action Result

        return (기본):
          {
            "success": bool,
            "heard_text": str,
            "code": int,      # 0 OK / 1 TIMEOUT / 2 MISMATCH / 3 CANCELED
            "reason": str
          }
        """
        expected = (expected or "").strip()
        heard_text = (heard_text or "").strip()

        #################### exceptions ####################
        # if nothing heard
        if not heard_text:
            r = {"success": False, "heard_text": heard_text,
                "code": CODE_TIMEOUT, "reason": "empty heard_text"}
            return r
        if not expected:
            return {"success": False, "heard_text": heard_text,
                "code": CODE_MISMATCH, "reason": "empty expected"}
        #################### exceptions ####################

        
        # LLM call
        try:
            raw = self.lang_chain.invoke({"expected": expected, "heard_text": heard_text}).content.strip()
        except Exception as e:
            r = {"success": False, "heard_text": heard_text, "code": CODE_TIMEOUT, "reason": f"LLM invoke error: {e}"}
            return r
        
        # PArsing
        temp = re.match(
            r"^(OK|MISMATCH)\s*/\s*extracted=(.*?)\s*/\s*reason=(.*?)\s*$",
            raw
        )

        #################### exceptions ####################
        if not temp:
            r = {"success": False, "heard_text": heard_text, "code": CODE_MISMATCH, "reason": "LLM output format invalid"}
            return r
        #################### exceptions ####################

        verdict = temp.group(1).strip()
        _extracted = temp.group(2).strip()  # whenever i need
        reason = temp.group(3).strip()

        if verdict == "OK":
            r = {"success": True, "heard_text": heard_text, "code": CODE_OK, "reason": reason}
        else:
            # MISMATCH
            r = {"success": False, "heard_text": heard_text, "code": CODE_MISMATCH, "reason": reason}

        return r


def main():
    verifier = GetKeyword()
    expected = "아졸리"

    tests = [
        "아졸리",          # OK 기대
        "아 졸 리",        # OK 기대
        "아졸리입니다",     # OK 기대
        "아 졸리다",       # MISMATCH/UNKNOWN 기대 (억지매칭 금지)
        "졸리다",          # MISMATCH 기대
        "아우 졸리다",     # MISMATCH 기대
    ]

    for test in tests:
        r = verifier.verify(expected=expected, heard_text=test)
        print(f"\nexpected={expected} | heard_text={test}")
        print(r)


if __name__ == "__main__":
    main()